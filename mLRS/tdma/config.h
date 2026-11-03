//*******************************************************
// Swarm C2 TDMA firmware
// GPL3
//*******************************************************
// config: role, node ID, radio settings, frame constants
//*******************************************************
// Spec: docs/phases-2-4.md
// Role and node ID are compile-time defines, set by tools/run_make_tdma.py:
//   TDMA_ROLE     TDMA_ROLE_GROUND or TDMA_ROLE_DRONE
//   TDMA_NODE_ID  0 for the ground node, 1..20 for drone nodes
//*******************************************************
#ifndef TDMA_CONFIG_H
#define TDMA_CONFIG_H
#pragma once


#define TDMA_ROLE_GROUND        0
#define TDMA_ROLE_DRONE         1

#ifndef TDMA_ROLE
  #error TDMA_ROLE not defined !
#endif
#if (TDMA_ROLE != TDMA_ROLE_GROUND) && (TDMA_ROLE != TDMA_ROLE_DRONE)
  #error TDMA_ROLE must be TDMA_ROLE_GROUND or TDMA_ROLE_DRONE !
#endif
#ifndef TDMA_NODE_ID
  #error TDMA_NODE_ID not defined !
#endif
#if (TDMA_ROLE == TDMA_ROLE_GROUND) && (TDMA_NODE_ID != 0)
  #error the ground node must have TDMA_NODE_ID 0 !
#endif
#if (TDMA_ROLE == TDMA_ROLE_DRONE) && ((TDMA_NODE_ID < 1) || (TDMA_NODE_ID > 20))
  #error drone nodes must have TDMA_NODE_ID 1..20 !
#endif


//-- radio

#define TDMA_FREQ_HZ            869525000UL  // 869.4-869.65 MHz; 250 kHz BW fills the whole sub-band, no edge margin
#define TDMA_SF                 8
#define TDMA_BW_KHZ             250
#define TDMA_CR                 5            // 4/5
#define TDMA_PREAMBLE_SYMS      8
#define TDMA_TX_POWER_DBM_MAX   24           // bench cap for phases 2-4; field max 27 dBm ERP incl. antenna gain

#ifndef TDMA_TX_POWER_DBM
  #define TDMA_TX_POWER_DBM     17           // Matek minimum at 868 MHz; default for bench runs
#endif

static_assert(TDMA_TX_POWER_DBM <= TDMA_TX_POWER_DBM_MAX, "TDMA_TX_POWER_DBM above the 24 dBm bench cap");


//-- frame

#define TDMA_FRAME_US           2000000UL
#define TDMA_BEACON_START_US    0
#define TDMA_SLOT1_START_US     150000UL     // slots are numbered 1-20
#define TDMA_SLOT_US            80000UL
#define TDMA_NUM_SLOTS          20
#define TDMA_SPARE_START_US     1750000UL

// Airtime at SF8 / 250 kHz / CR 4/5, explicit header, CRC on:
//   30-byte slot packet ~61.7 ms (18.3 ms guard), 40-byte beacon ~77.1 ms
//   HeaderValid ~20.7 ms after TX start (12.25 preamble + 8 header symbols)
#define TDMA_BEACON_RX_LEAD_US     5000UL   // open RX this early (TRACK)
#define TDMA_BEACON_RX_TIMEOUT_US  35000UL  // lead + ~21 ms to HeaderValid + margin

#define TDMA_HOLDOVER_FRAMES    30           // then go silent
#define TDMA_DUTY_LIMIT_PPM     100000UL     // 10% per hour

// Measured in Phase 2, not guessed:
#define T_RX_HDRVALID_OFFSET_US 0            // TX start -> receiver HeaderValid
#define T_TX_LATENCY_US         0            // SetTx command -> RF on air


//-- Phase 2 timing test (tasks 2.1-2.4), replaced by the TDMA frame in Phase 3

#define TDMA_TEST_PERIOD_US     1000000UL    // ground transmits on this exact TIM2 grid
#define TDMA_TEST_LEN           30           // same size as a Phase 3 slot packet
#define TDMA_TEST_ARM_LEAD_US   30000UL      // load the packet this long before its on-air time

// one 30-byte packet is ~61.7 ms on air: ~6.2% duty at 1 s.
// (Task 2.5 in the spec says every 500 ms: that is ~12.4%, over the 10% limit.)
static_assert(TDMA_TEST_PERIOD_US >= 700000UL, "test period too short for the 10% duty limit");


#define TIM2_IRQ_PRIORITY       1            // timebase, DIO1 capture, scheduled SetTx
#define SX_DIO_EXTI_IRQ_PRIORITY 1           // hal's EXTI helper, unused (EXTI fallback sets its own, = TIM2)
#define UARTB_IRQ_PRIORITY      11           // TX1/RX1 log
// USB: USBD_IRQ_PRIORITY = 14, set on the command line by tools/run_make_tdma.py
// SysTick: 15


#endif // TDMA_CONFIG_H
