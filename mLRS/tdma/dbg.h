//*******************************************************
// Swarm C2 TDMA firmware
// GPL3
//*******************************************************
// dbg: debug pin pulses, TX-window pin, CSV log on TX1
//*******************************************************
// Pins (see docs/phases-2-4.md, Firmware structure and conventions):
//   TX2 (PB3)  debug pin, pulse width identifies the event
//   LR1 (PA3)  TX-window pin, high while the amp is switched to transmit
//   TX1 (PA9)  UART1 115200 8N1, line-based CSV log
//*******************************************************
#ifndef TDMA_DBG_H
#define TDMA_DBG_H
#pragma once

#include <stdio.h>
#include <stdarg.h>


#define DBG_PIN                 IO_PB3  // TX2
#define DBG_TXWINDOW_PIN        IO_PA3  // LR1

// pulse widths, us
#define DBG_PULSE_FRAME_ZERO    10
#define DBG_PULSE_HDRVALID      20
#define DBG_PULSE_SETTX         40
#define DBG_PULSE_TXDONE        80


void dbg_init(void)
{
    gpio_init(DBG_PIN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
    gpio_init(DBG_TXWINDOW_PIN, IO_MODE_OUTPUT_PP_LOW, IO_SPEED_VERYFAST);
}

// blocking; replaced by timer output compare in Phase 2
void dbg_pulse(uint16_t width_us)
{
    gpio_high(DBG_PIN);
    delay_us(width_us);
    gpio_low(DBG_PIN);
}

void dbg_txwindow_high(void) { gpio_high(DBG_TXWINDOW_PIN); }
void dbg_txwindow_low(void) { gpio_low(DBG_TXWINDOW_PIN); }


// one CSV line, '\n' appended
void dbg_log(const char* fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf) - 1, fmt, args);
    va_end(args);
    if (n < 0) return;
    if (n > (int)sizeof(buf) - 2) n = sizeof(buf) - 2;
    buf[n++] = '\n';
    uartb_putbuf((uint8_t*)buf, n);
}


#endif // TDMA_DBG_H
