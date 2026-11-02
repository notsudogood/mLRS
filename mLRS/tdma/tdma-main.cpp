//*******************************************************
// Swarm C2 TDMA firmware
// GPL3
//*******************************************************
// main: prerequisite link test (phases 0-1)
//*******************************************************
// Two units exchange fixed 30-byte packets at 869.525 MHz, SF8, 250 kHz, CR 4/5.
// The ground node (TDMA_ROLE_GROUND) sends a ping once per second and waits
// for a pong. A drone node (TDMA_ROLE_DRONE) listens continuously and answers
// each ping. Both log RSSI and SNR as CSV on TX1 (115200 8N1).
// Use exactly one drone node with this test: drones answer at once and two
// would collide. The TDMA frame replaces this in Phase 3.
//*******************************************************

// priorities in one place, as mLRS does (lower = higher priority, SysTick at 15)
#define UARTB_IRQ_PRIORITY          11 // TX1/RX1 log
#define SX_DIO_EXTI_IRQ_PRIORITY    13 // not enabled yet, DIO1 is polled

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

// USB-C CDC (virtual COM port), the primary log output; USB IRQ priority is 0 (usbd_conf.h)
#ifndef STDSTM32_USE_USB
  #error build with tools/run_make_tdma.py, it sets STDSTM32_USE_USB !
#endif
#include "../modules/stm32-usb-device/stdstm32-usb-vcp.h"

#include "config.h"
#include "dbg.h"
#include "radio.h"


tRadio radio;


//-------------------------------------------------------
// Packet
//-------------------------------------------------------

#define PKT_MAGIC0              'T'
#define PKT_MAGIC1              'D'
#define PKT_VERSION             1

typedef enum {
    PKT_TYPE_PING = 1,
    PKT_TYPE_PONG = 2,
} PKT_TYPE_ENUM;

typedef struct {
    uint8_t magic[2];
    uint8_t version;
    uint8_t type;
    uint8_t node_id;        // sender
    uint32_t seq;           // ping sequence number, echoed in the pong
    int8_t remote_rssi;     // pong only: RSSI of the ping at the drone
    int8_t remote_snr;      // pong only: SNR of the ping at the drone
    uint8_t padding[TDMA_PING_LEN - 11];
} __attribute__((packed)) tPingPacket;

static_assert(sizeof(tPingPacket) == TDMA_PING_LEN, "tPingPacket must be TDMA_PING_LEN bytes");


void pkt_build(tPingPacket* pkt, uint8_t type, uint32_t seq, int8_t remote_rssi, int8_t remote_snr)
{
    pkt->magic[0] = PKT_MAGIC0;
    pkt->magic[1] = PKT_MAGIC1;
    pkt->version = PKT_VERSION;
    pkt->type = type;
    pkt->node_id = TDMA_NODE_ID;
    pkt->seq = seq;
    pkt->remote_rssi = remote_rssi;
    pkt->remote_snr = remote_snr;
    for (uint8_t i = 0; i < sizeof(pkt->padding); i++) pkt->padding[i] = (uint8_t)(i + seq);
}


bool pkt_valid(tPingPacket* pkt, uint8_t len, uint8_t type)
{
    if (len != sizeof(tPingPacket)) return false;
    if (pkt->magic[0] != PKT_MAGIC0 || pkt->magic[1] != PKT_MAGIC1) return false;
    if (pkt->version != PKT_VERSION) return false;
    if (pkt->type != type) return false;
    return true;
}


//-------------------------------------------------------
// Helpers
//-------------------------------------------------------

// waits for any of the irq bits in mask, or tmo_ms; returns the irq bits seen (0 on timeout)
uint16_t wait_irq(uint16_t mask, uint16_t tmo_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint16_t seen = 0;
    while ((HAL_GetTick() - t0) < tmo_ms) {
        if (!radio.IrqPending()) continue;
        uint16_t irq = radio.GetAndClearIrq();
        if (irq & SX126X_IRQ_HEADER_VALID) dbg_pulse(DBG_PULSE_HDRVALID);
        if (irq & SX126X_IRQ_TX_DONE) dbg_pulse(DBG_PULSE_TXDONE);
        seen |= irq;
        if (seen & mask) return seen;
    }
    return 0;
}


// sends and waits for TxDone; on failure forces the amp back to receive
bool send_blocking(tPingPacket* pkt)
{
    radio.Send((uint8_t*)pkt, sizeof(tPingPacket));
    dbg_pulse(DBG_PULSE_SETTX);
    if (wait_irq(SX126X_IRQ_TX_DONE, 150)) return true; // ~62 ms airtime
    radio.Idle();
    sx_amp_receive();
    dbg_txwindow_low();
    return false;
}


//-------------------------------------------------------
// Ground node: ping, wait for pong
//-------------------------------------------------------

#if TDMA_ROLE == TDMA_ROLE_GROUND

void loop_main(void)
{
    uint32_t seq = 0;
    uint32_t tlast_ms = HAL_GetTick();
    tPingPacket pkt;
    tRadioRxStatus st;

    dbg_log("# columns: event,seq,node,rssi,snr,remote_rssi,remote_snr,crc_ok");

    while (1) {
        if ((HAL_GetTick() - tlast_ms) < TDMA_PING_PERIOD_MS) continue;
        tlast_ms += TDMA_PING_PERIOD_MS;
        seq++;

        pkt_build(&pkt, PKT_TYPE_PING, seq, 0, 0);
        if (!send_blocking(&pkt)) {
            led_red_on();
            dbg_log("tx_fail,%lu,,,,,,", seq);
            continue;
        }

        radio.StartRx(TDMA_PONG_TIMEOUT_MS);
        uint16_t irq = wait_irq(SX126X_IRQ_RX_DONE | SX126X_IRQ_RX_TX_TIMEOUT, TDMA_PONG_TIMEOUT_MS + 50);
        if (!(irq & SX126X_IRQ_RX_DONE)) {
            radio.Idle();
            led_red_toggle();
            led_green_off();
            dbg_log("timeout,%lu,,,,,,", seq);
            continue;
        }

        radio.ReadPacket((uint8_t*)&pkt, sizeof(pkt), &st, irq);
        radio.Idle();
        if (!st.crc_ok) {
            dbg_log("crc_err,%lu,,%d,%d,,,0", seq, st.rssi, st.snr);
            continue;
        }
        if (!pkt_valid(&pkt, st.len, PKT_TYPE_PONG) || pkt.seq != seq) {
            dbg_log("bad_pkt,%lu,,%d,%d,,,1", seq, st.rssi, st.snr);
            continue;
        }

        led_red_off();
        led_green_toggle();
        dbg_log("pong,%lu,%u,%d,%d,%d,%d,1", seq, pkt.node_id, st.rssi, st.snr, pkt.remote_rssi, pkt.remote_snr);
    }
}

#endif


//-------------------------------------------------------
// Drone node: listen, answer each ping
//-------------------------------------------------------

#if TDMA_ROLE == TDMA_ROLE_DRONE

void loop_main(void)
{
    tPingPacket pkt;
    tRadioRxStatus st;
    uint32_t tlasttx_ms = HAL_GetTick() - TDMA_PING_PERIOD_MS;
    uint32_t tlastrx_ms = HAL_GetTick();

    dbg_log("# columns: event,seq,node,rssi,snr,crc_ok");

    radio.StartRx(0);

    while (1) {
        if ((HAL_GetTick() - tlastrx_ms) > 3 * TDMA_PING_PERIOD_MS) { // no ping for a while
            led_red_on();
            led_green_off();
        }

        uint16_t irq = wait_irq(SX126X_IRQ_RX_DONE, 10);
        if (!(irq & SX126X_IRQ_RX_DONE)) continue;

        radio.ReadPacket((uint8_t*)&pkt, sizeof(pkt), &st, irq);
        if (!st.crc_ok) {
            dbg_log("crc_err,,,%d,%d,0", st.rssi, st.snr);
            radio.StartRx(0);
            continue;
        }
        if (!pkt_valid(&pkt, st.len, PKT_TYPE_PING)) {
            dbg_log("bad_pkt,,,%d,%d,1", st.rssi, st.snr);
            radio.StartRx(0);
            continue;
        }

        tlastrx_ms = HAL_GetTick();
        led_red_off();
        led_green_toggle();
        dbg_log("ping,%lu,%u,%d,%d,1", pkt.seq, pkt.node_id, st.rssi, st.snr);

        // duty-cycle guard: never answer faster than the ground's ping period allows
        if ((HAL_GetTick() - tlasttx_ms) < 700) {
            dbg_log("skip_duty,%lu,,,,", pkt.seq);
            radio.StartRx(0);
            continue;
        }

        uint32_t seq = pkt.seq;
        pkt_build(&pkt, PKT_TYPE_PONG, seq, st.rssi, st.snr);
        tlasttx_ms = HAL_GetTick();
        if (!send_blocking(&pkt)) {
            dbg_log("tx_fail,%lu,,,,", seq);
        }
        radio.StartRx(0);
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

    dbg_log("# swarm-c2 tdma prereq, role %s, node %u",
            (TDMA_ROLE == TDMA_ROLE_GROUND) ? "ground" : "drone", TDMA_NODE_ID);

    if (!radio.Init()) {
        dbg_log("# ERROR: SX1262 not responding");
        while (1) { led_red_toggle(); delay_ms(100); } // fast red blink, radio stays off
    }
    radio.Configure();

    dbg_log("# freq %lu Hz, SF%u, BW %u kHz, CR 4/%u, power %d dBm (cap %d)",
            TDMA_FREQ_HZ, TDMA_SF, TDMA_BW_KHZ, TDMA_CR, radio.Power_dbm(), TDMA_TX_POWER_DBM_MAX);

    led_red_off();
    loop_main();
}
