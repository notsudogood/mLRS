//*******************************************************
// Swarm C2 TDMA firmware
// GPL3
//*******************************************************
// radio: thin wrapper over the mLRS SX126x driver (sx12xx-lib)
//*******************************************************
// Keeps the mLRS hardware layer (hal, SPI, amp switching, TCXO setup)
// and uses none of the mLRS link logic.
//
// Timestamps (TIM2 us, see timebase.h), taken in the TIM2 interrupt:
//   radio_t_hdrvalid  DIO1 rising edge of the first IRQ of a received packet (HeaderValid)
//   radio_t_settx     instant SetTx was issued by a scheduled transmit
//   radio_t_txdone    DIO1 rising edge of TxDone
// SPI rule: while a transmit is armed (RADIO_TX_ARMED) the TIM2 interrupt
// owns the SPI bus; the main loop must not call any radio function.
//*******************************************************
#ifndef TDMA_RADIO_H
#define TDMA_RADIO_H
#pragma once


#if TDMA_SF != 8 || TDMA_BW_KHZ != 250 || TDMA_CR != 5
  #error radio.h maps only SF8 / 250 kHz / CR 4/5, extend the mapping first !
#endif


typedef struct {
    int8_t rssi;    // dBm
    int8_t snr;     // dB
    uint8_t len;
    bool crc_ok;
} tRadioRxStatus;


typedef enum {
    RADIO_IDLE = 0,
    RADIO_RX,
    RADIO_TX_ARMED,     // TxAt() done, SetTx pending in tb_at()
    RADIO_TX_ON,        // SetTx issued, waiting for TxDone
} RADIO_STATE_ENUM;

volatile uint8_t radio_state = RADIO_IDLE;
volatile bool radio_hdr_seen = false;
volatile uint32_t radio_t_hdrvalid = 0;
volatile uint32_t radio_t_settx = 0;
volatile uint32_t radio_t_txdone = 0;
volatile uint32_t radio_dio_edges = 0;

void radio_settx_isr(void); // below the class


class tRadio : public Sx126xDriverBase
{
  public:

    //-- interface to SPI peripheral, as in mLRS Sx126xDriver

    void WaitOnBusy(void) override
    {
        while (sx_busy_read()) { __NOP(); };
    }

    void SpiSelect(void) override
    {
        spi_select();
        delay_ns(50); // t6 = 15 ns, t1 = 32 ns
    }

    void SpiDeselect(void) override
    {
        delay_ns(50); // t8 = 31.25 ns
        spi_deselect();
    }

    void SpiTransfer(uint8_t* dataout, uint8_t* datain, uint8_t len) override
    {
        spi_transfer(dataout, datain, len);
    }

    void SpiRead(uint8_t* datain, uint8_t len) override
    {
        spi_read(datain, len);
    }

    void SpiWrite(uint8_t* dataout, uint8_t len) override
    {
        spi_write(dataout, len);
    }

    //-- init

    // returns false if the SX1262 does not answer
    bool Init(void)
    {
        spi_init();
        spi_setnop(0x00);
        sx_init_gpio(); // TX_EN and RX_EN low: amp off, nothing can transmit
        // DIO1 is polled in phase 0-1 code; the EXTI/input-capture path comes in Phase 2

        delay_ms(300); // SX126x boot, as mLRS does
        gpio_low(SX_RESET);
        delay_ms(5);
        gpio_high(SX_RESET);
        delay_ms(50);
        WaitOnBusy();

        SetStandby(SX126X_STDBY_CONFIG_STDBY_RC);
        delay_us(1000);

        uint16_t rev = GetFirmwareRev();
        return (rev != 0) && (rev != 0xFFFF);
    }

    void Configure(void)
    {
        SetPacketType(SX126X_PACKET_TYPE_LORA);
        SetTxClampConfig(); // workaround 15.2.2
        ClearDeviceError(); // XOSC_START_ERR is raised before TCXO setup
        SetDio3AsTcxoControl(SX126X_DIO3_OUTPUT_1_8, 250); // mLRS default for this target
        CalibrateImage(SX126X_CAL_IMG_863_MHZ_1, SX126X_CAL_IMG_863_MHZ_2);
        SetAutoFs(true);
        SetRxGain(SX126X_RX_GAIN_BOOSTED_GAIN);
        SetOverCurrentProtection(SX126X_OCP_CONFIGURATION_140_MA);
#ifdef SX_USE_PA_CONFIG_10_DBM
        SetPaConfig_10dbm(); // set by hal-power-pa.h for the mR900-30
#else
        SetPaConfig_22dbm();
#endif
        SetPower_dbm(TDMA_TX_POWER_DBM);

        SetRfFrequency(SX126X_FREQ_MHZ_TO_REG((double)TDMA_FREQ_HZ * 1.0E-6));
        SetModulationParams(SX126X_LORA_SF8, SX126X_LORA_BW_250, SX126X_LORA_CR_4_5);
        SetPacketParams(TDMA_PREAMBLE_SYMS, SX126X_LORA_HEADER_EXPLICIT, 255, SX126X_LORA_CRC_ENABLE, SX126X_LORA_IQ_NORMAL);
        SetBufferBaseAddress(0, 0);

        SetDioIrqParams(SX126X_IRQ_ALL,
                        SX126X_IRQ_HEADER_VALID | SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE |
                        SX126X_IRQ_CRC_ERROR | SX126X_IRQ_RX_TX_TIMEOUT,
                        SX126X_IRQ_NONE,
                        SX126X_IRQ_NONE);
        ClearIrqStatus(SX126X_IRQ_ALL);
        SetFs();
    }

    // the power cap: anything above TDMA_TX_POWER_DBM_MAX is clamped, never passed on
    void SetPower_dbm(int8_t power_dbm)
    {
        if (power_dbm > TDMA_TX_POWER_DBM_MAX) power_dbm = TDMA_TX_POWER_DBM_MAX;
        int8_t sx_power;
        sx126x_rfpower_calc(power_dbm, &sx_power, &actual_power_dbm, SX_FHSS_FREQUENCY_BAND_868_MHZ);
        if (actual_power_dbm > TDMA_TX_POWER_DBM_MAX) while(1){} // must not happen, stop rather than transmit
        SetTxParams(sx_power, SX126X_RAMPTIME_40_US);
    }

    int8_t Power_dbm(void) { return actual_power_dbm; }

    //-- TX / RX

    // Scheduled transmit (task 2.4): puts the radio in STDBY_XOSC with the
    // packet loaded now, then the TIM2 interrupt issues SetTx at
    // t_on_air - T_TX_LATENCY_US. Returns false and transmits nothing if that
    // time is too close or already past.
    bool TxAt(uint32_t t_on_air, uint8_t* data, uint8_t len)
    {
        if (radio_state == RADIO_TX_ARMED || radio_state == RADIO_TX_ON) return false;
        SetStandby(SX126X_STDBY_CONFIG_STDBY_XOSC); // TCXO stays on: SetTx starts without the TCXO delay
        SetPacketParams(TDMA_PREAMBLE_SYMS, SX126X_LORA_HEADER_EXPLICIT, len, SX126X_LORA_CRC_ENABLE, SX126X_LORA_IQ_NORMAL);
        WriteBuffer(0, data, len);
        ClearIrqStatus(SX126X_IRQ_ALL);
        radio_hdr_seen = false;
        radio_state = RADIO_TX_ARMED;
        if (!tb_at(t_on_air - T_TX_LATENCY_US, radio_settx_isr)) {
            radio_state = RADIO_IDLE;
            Idle();
            return false;
        }
        return true;
    }

    // Aborts a TX that did not end (no TxDone): amp back to receive, radio to FS.
    void AbortTx(void)
    {
        tb_at_cancel();
        radio_state = RADIO_IDLE;
        Idle();
        sx_amp_receive();
        dbg_txwindow_low();
    }

    // tmo_ms = 0: continuous RX
    void StartRx(uint16_t tmo_ms)
    {
        SetPacketParams(TDMA_PREAMBLE_SYMS, SX126X_LORA_HEADER_EXPLICIT, 255, SX126X_LORA_CRC_ENABLE, SX126X_LORA_IQ_NORMAL);
        ClearIrqStatus(SX126X_IRQ_ALL);
        radio_hdr_seen = false;
        radio_state = RADIO_RX;
        sx_amp_receive();
        SetRx((tmo_ms == 0) ? 0xFFFFFF : (uint32_t)tmo_ms * 64); // 15.625 us steps; 0xFFFFFF = continuous
    }

    // ready for the next packet in continuous RX: next DIO1 edge is a new HeaderValid
    void RxRearm(void)
    {
        radio_hdr_seen = false;
    }

    void Idle(void)
    {
        SetFs();
        ClearIrqStatus(SX126X_IRQ_ALL);
        if (radio_state != RADIO_TX_ARMED) radio_state = RADIO_IDLE;
    }

    // DIO1 level, polled
    bool IrqPending(void)
    {
        return gpio_read_activehigh(SX_DIO);
    }

    uint16_t GetAndClearIrq(void)
    {
        uint16_t irq = GetIrqStatus();
        ClearIrqStatus(irq);
        if (irq & SX126X_IRQ_TX_DONE) {
            dbg_txwindow_low();
            sx_amp_receive(); // amp off TX as soon as the packet is out
            radio_state = RADIO_IDLE;
        }
        return irq;
    }

    // call after RxDone; copies at most maxlen bytes
    void ReadPacket(uint8_t* data, uint8_t maxlen, tRadioRxStatus* status, uint16_t irq)
    {
        uint8_t len, start;
        GetRxBufferStatus(&len, &start);
        if (len > maxlen) len = maxlen;
        ReadBuffer(start, data, len);

        int16_t rssi;
        int8_t snr;
        GetPacketStatus(&rssi, &snr);
        if (rssi > -1) rssi = -1;
        if (rssi < -127) rssi = -127;

        status->rssi = rssi;
        status->snr = snr;
        status->len = len;
        status->crc_ok = !(irq & SX126X_IRQ_CRC_ERROR);
    }

  private:
    int8_t actual_power_dbm;
};


extern tRadio radio;


// TIM2 interrupt, at t_on_air - T_TX_LATENCY_US (task 2.4)
void radio_settx_isr(void)
{
    if (radio_state != RADIO_TX_ARMED) return;
    sx_amp_transmit();
    dbg_txwindow_high();
    radio_state = RADIO_TX_ON;
    radio_t_settx = TIM2->CNT;
    dbg_pulse_start(DBG_PULSE_SETTX); // rising edge marks the start of the SetTx command
    radio.SetTx(0); // no timeout; TxDone ends it
}


// TIM2 interrupt, DIO1 rising edge (task 2.3)
void radio_dio1_isr(uint32_t t)
{
    radio_dio_edges++;
    if (radio_state == RADIO_TX_ON) {
        radio_t_txdone = t;
        dbg_pulse_start(DBG_PULSE_TXDONE);
    } else
    if (radio_state == RADIO_RX && !radio_hdr_seen) { // first IRQ of a packet is HeaderValid
        radio_hdr_seen = true;
        radio_t_hdrvalid = t;
        dbg_pulse_start(DBG_PULSE_HDRVALID);
    }
}


#endif // TDMA_RADIO_H
