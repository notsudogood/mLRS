//*******************************************************
// Swarm C2 TDMA firmware
// GPL3
//*******************************************************
// main: Phase 2 timing test (tasks 2.1-2.4)
//*******************************************************
// Ground node (TDMA_ROLE_GROUND): transmits a 30-byte test packet with a
// scheduled transmit (radio.TxAt) every TDMA_TEST_PERIOD_US on an exact TIM2
// grid, and logs when SetTx was issued and when TxDone came.
// Drone node (TDMA_ROLE_DRONE): listens continuously, never transmits, and
// logs the hardware-captured HeaderValid time of every packet plus the
// interval to the previous one, i.e. the clock error of the drone's TIM2
// relative to the ground's TIM2.
// All times are TIM2 microseconds (timebase.h). CSV goes to USB-C and TX1.
// Any number of drone nodes can listen at the same time.
//*******************************************************

#include "config.h" // first: it sets the interrupt priorities used by the libraries below

#include "../Common/common_conf.h"
#include "../Common/common_types.h" // includes setup_types.h
#include "../modules/sx12xx-lib/src/sx126x.h"

#include "../Common/hal/glue.h"
#include "../modules/stm32ll-lib/src/stdstm32.h"
#include "../modules/stm32ll-lib/src/stdstm32-peripherals.h"
#include "../modules/stm32ll-lib/src/stdstm32-mcu.h"
#include "../Common/thirdparty/stdstm32-exti.h"
#include "../Common/hal/hal.h"
#include "../modules/stm32ll-lib/src/stdstm32-delay.h" // these are dependent on hal
#include "../modules/stm32ll-lib/src/stdstm32-spi.h"

#undef UARTB_BAUD
#define UARTB_BAUD                  115200
#include "../modules/stm32ll-lib/src/stdstm32-uartb.h"

// USB-C CDC (virtual COM port), the primary log output
#ifndef STDSTM32_USE_USB
  #error build with tools/run_make_tdma.py, it sets STDSTM32_USE_USB !
#endif
#if !defined USBD_IRQ_PRIORITY || USBD_IRQ_PRIORITY <= TIM2_IRQ_PRIORITY
  #error USBD_IRQ_PRIORITY must be set below TIM2 (higher number), see tools/run_make_tdma.py !
#endif
#include "../modules/stm32-usb-device/stdstm32-usb-vcp.h"

#include "dbg.h"
#include "timebase.h"
#include "radio.h"


tRadio radio;

uint32_t cnt_tx = 0, cnt_rx_ok = 0, cnt_rx_err = 0, cnt_late = 0; // reported by heartbeat()


//-------------------------------------------------------
// Packet
//-------------------------------------------------------

#define PKT_MAGIC0              'T'
#define PKT_MAGIC1              'D'
#define PKT_VERSION             2

typedef enum {
    PKT_TYPE_TIMING_TEST = 3,
} PKT_TYPE_ENUM;

typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t type;
    uint8_t node_id;        // sender
    uint32_t seq;           // increments once per test period, also over late (skipped) periods
    uint32_t t_plan;        // sender's planned on-air time, sender's TIM2 us
    uint8_t padding[TDMA_TEST_LEN - 13];
} __attribute__((packed)) tTestPacket;

static_assert(sizeof(tTestPacket) == TDMA_TEST_LEN, "tTestPacket must be TDMA_TEST_LEN bytes");


void pkt_build(tTestPacket* pkt, uint32_t seq, uint32_t t_plan)
{
    pkt->magic[0] = PKT_MAGIC0;
    pkt->magic[1] = PKT_MAGIC1;
    pkt->version = PKT_VERSION;
    pkt->type = PKT_TYPE_TIMING_TEST;
    pkt->node_id = TDMA_NODE_ID;
    pkt->seq = seq;
    pkt->t_plan = t_plan;
    for (uint8_t i = 0; i < sizeof(pkt->padding); i++) pkt->padding[i] = (uint8_t)(i + seq);
}


bool pkt_valid(tTestPacket* pkt, uint8_t len)
{
    if (len != sizeof(tTestPacket)) return false;
    if (pkt->magic[0] != PKT_MAGIC0 || pkt->magic[1] != PKT_MAGIC1) return false;
    if (pkt->version != PKT_VERSION) return false;
    if (pkt->type != PKT_TYPE_TIMING_TEST) return false;
    return true;
}


//-------------------------------------------------------
// Helpers
//-------------------------------------------------------

// status line every 10 s, so a logger opened late still sees role, node and counters
#define HEARTBEAT_PERIOD_MS     10000

uint32_t heartbeat_tlast_ms = 0;

void heartbeat(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - heartbeat_tlast_ms) < HEARTBEAT_PERIOD_MS) return;
    heartbeat_tlast_ms = now;
    dbg_log("# alive,uptime_s=%lu,role=%s,node=%u,power_dbm=%d,tx=%lu,late=%lu,rx_ok=%lu,rx_err=%lu,tb_us=%lu",
            now / 1000, (TDMA_ROLE == TDMA_ROLE_GROUND) ? "ground" : "drone", TDMA_NODE_ID,
            radio.Power_dbm(), cnt_tx, cnt_late, cnt_rx_ok, cnt_rx_err, tb_now());
}


//-------------------------------------------------------
// Ground node: scheduled transmit on an exact grid (tasks 2.1, 2.4)
//-------------------------------------------------------
// SetTx is issued every TDMA_TEST_PERIOD_US exactly (1 000 000 TIM2 ticks), so
// the 40 us SetTx pulses on the debug pin double as the task 2.1 check:
// their spacing on the logic analyser is 1.000 s +- the board's clock error.

#if TDMA_ROLE == TDMA_ROLE_GROUND

void loop_main(void)
{
    tTestPacket pkt;
    uint32_t seq = 0;
    uint32_t t_next = tb_now() + 2 * TDMA_TEST_PERIOD_US;
    uint32_t t_armed = 0;
    uint32_t seq_armed = 0;
    bool armed = false;

    dbg_log("# columns: event,seq,t_plan_us,t_settx_us,t_txdone_us,settx_err_us,txdone_minus_settx_us");

    while (1) {
        heartbeat();

        if (!armed) {
            if (tb_diff(t_next, tb_now()) > (int32_t)TDMA_TEST_ARM_LEAD_US) continue;
            seq++;
            pkt_build(&pkt, seq, t_next);
            if (radio.TxAt(t_next, (uint8_t*)&pkt, sizeof(pkt))) {
                armed = true;
                t_armed = t_next;
                seq_armed = seq;
            } else {
                cnt_late++;
                led_red_on();
                dbg_log("late,%lu,%lu,,,,", seq, t_next);
            }
            t_next += TDMA_TEST_PERIOD_US; // the grid never moves, a late period is skipped
            continue;
        }

        // armed: the TIM2 interrupt owns the SPI bus until it has issued SetTx
        if (radio_state == RADIO_TX_ARMED) continue;

        if (radio_state == RADIO_TX_ON && !radio.IrqPending()) {
            if (tb_diff(tb_now(), radio_t_settx) > 150000) { // ~62 ms airtime, TxDone never came
                radio.AbortTx();
                armed = false;
                led_red_on();
                dbg_log("tx_fail,%lu,%lu,%lu,,,", seq_armed, t_armed, radio_t_settx);
            }
            continue;
        }

        uint16_t irq = radio.GetAndClearIrq();
        if (!(irq & SX126X_IRQ_TX_DONE)) continue;

        armed = false;
        cnt_tx++;
        led_red_off();
        led_green_toggle();
        uint32_t t_settx = radio_t_settx;
        uint32_t t_txdone = radio_t_txdone;
        dbg_log("tx,%lu,%lu,%lu,%lu,%ld,%ld",
                seq_armed, t_armed, t_settx, t_txdone,
                tb_diff(t_settx, t_armed - T_TX_LATENCY_US), tb_diff(t_txdone, t_settx));
    }
}

#endif


//-------------------------------------------------------
// Drone node: listen, timestamp HeaderValid (tasks 2.2, 2.3)
//-------------------------------------------------------
// interval_us: HeaderValid to HeaderValid, drone TIM2
// err_us:      interval_us - (seq difference) x TDMA_TEST_PERIOD_US
// ppm_x10:     err_us relative to the expected interval, in 0.1 ppm;
//              positive = the drone's clock runs fast relative to the ground's

#if TDMA_ROLE == TDMA_ROLE_DRONE

void loop_main(void)
{
    tTestPacket pkt;
    tRadioRxStatus st;
    uint32_t prev_seq = 0;
    uint32_t prev_t = 0;
    uint32_t tlastrx_ms = HAL_GetTick();

    dbg_log("# columns: event,seq,t_hdrvalid_us,interval_us,err_us,ppm_x10,rssi,snr,crc_ok");

    radio.StartRx(0);

    while (1) {
        heartbeat();

        if ((HAL_GetTick() - tlastrx_ms) > 3 * (TDMA_TEST_PERIOD_US / 1000)) { // nothing heard for a while
            led_red_on();
            led_green_off();
        }

        // a HeaderValid without RxDone (should not happen): re-arm after 200 ms
        if (radio_hdr_seen && tb_diff(tb_now(), radio_t_hdrvalid) > 200000 && !radio.IrqPending()) {
            radio.RxRearm();
        }

        if (!radio.IrqPending()) continue;

        uint16_t irq = radio.GetAndClearIrq(); // clearing HeaderValid drops DIO1, RxDone raises it again
        if (!(irq & SX126X_IRQ_RX_DONE)) continue;

        bool have_hdr = radio_hdr_seen;
        uint32_t t_hdr = radio_t_hdrvalid;
        radio.ReadPacket((uint8_t*)&pkt, sizeof(pkt), &st, irq);
        radio.RxRearm();

        if (!st.crc_ok) {
            cnt_rx_err++;
            dbg_log("crc_err,,%lu,,,,%d,%d,0", t_hdr, st.rssi, st.snr);
            continue;
        }
        if (!pkt_valid(&pkt, st.len) || !have_hdr) {
            cnt_rx_err++;
            dbg_log("bad_pkt,,%lu,,,,%d,%d,1", t_hdr, st.rssi, st.snr);
            continue;
        }

        cnt_rx_ok++;
        tlastrx_ms = HAL_GetTick();
        led_red_off();
        led_green_toggle();

        if (prev_seq != 0 && pkt.seq > prev_seq) {
            uint32_t dseq = pkt.seq - prev_seq;
            int32_t interval = tb_diff(t_hdr, prev_t);
            int64_t expected = (int64_t)dseq * TDMA_TEST_PERIOD_US;
            int32_t err_us = (int32_t)(interval - expected);
            int32_t ppm_x10 = (int32_t)(((int64_t)err_us * 10000000) / expected);
            dbg_log("rx,%lu,%lu,%ld,%ld,%ld,%d,%d,1", pkt.seq, t_hdr, interval, err_us, ppm_x10, st.rssi, st.snr);
        } else {
            dbg_log("rx,%lu,%lu,,,,%d,%d,1", pkt.seq, t_hdr, st.rssi, st.snr); // first packet, or ground restarted
        }
        prev_seq = pkt.seq;
        prev_t = t_hdr;
    }
}

#endif


//-------------------------------------------------------
// Init
//-------------------------------------------------------

// glue.h supplies main_main() { while(1) main_loop(); }; ours never returns
void main_loop(void)
{
    delay_init();
    leds_init();
    fan_init(); // fan off; it only runs at >= 27 dBm in mLRS
    dbg_init();
    uartb_init();
    usb_init();

    led_red_on();
    delay_ms(1500); // give the host time to enumerate the USB port, so the header lines are not lost

    dbg_log("# swarm-c2 tdma phase2 timing test, role %s, node %u",
            (TDMA_ROLE == TDMA_ROLE_GROUND) ? "ground" : "drone", TDMA_NODE_ID);

    if (!radio.Init()) {
        dbg_log("# ERROR: SX1262 not responding");
        while (1) { led_red_toggle(); delay_ms(100); } // fast red blink, radio stays off
    }
    radio.Configure();

    tb_init(); // after radio.Init(): takes PA15 (DIO1) over as TIM2_CH1 input capture
    tb_set_capture_callback(radio_dio1_isr);

    dbg_log("# freq %lu Hz, SF%u, BW %u kHz, CR 4/%u, power %d dBm (cap %d), period %lu us, timer %lu Hz, dio1 %s",
            TDMA_FREQ_HZ, TDMA_SF, TDMA_BW_KHZ, TDMA_CR, radio.Power_dbm(), TDMA_TX_POWER_DBM_MAX,
            TDMA_TEST_PERIOD_US, tb_timer_clock_hz(),
#ifdef TDMA_DIO1_USE_EXTI
            "exti");
#else
            "tim2_ch1_capture");
#endif

    led_red_off();
    loop_main();
}
