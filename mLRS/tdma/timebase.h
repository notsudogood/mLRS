//*******************************************************
// Swarm C2 TDMA firmware
// GPL3
//*******************************************************
// timebase: TIM2 free-running 32-bit counter at 1 MHz
//*******************************************************
// Every timestamp in the system is a uint32_t microsecond count from TIM2.
// Compare with tb_diff() (signed), so the ~71.6 min wrap is harmless.
//
// TIM2 channels:
//   CH1  input capture, SX1262 DIO1 on PA15 (AF1 = TIM2_CH1), rising edge
//        -> the capture time is the instant DIO1 rose, taken in hardware
//   CH3  output compare, one-shot scheduler tb_at(t, fn)
//   CH4  output compare, end of the current debug pulse (non-blocking pulses)
//   CH2  free (PB3 = TIM2_CH2 could later make pulses in hardware)
//
// Fallback: build with -D TDMA_DIO1_USE_EXTI to timestamp DIO1 with EXTI15
// and tb_now() in the handler instead (adds software latency, see task 2.5).
//
// TIM2 interrupt priority: TIM2_IRQ_PRIORITY (1), above USB (14) and the UARTs.
//*******************************************************
#ifndef TDMA_TIMEBASE_H
#define TDMA_TIMEBASE_H
#pragma once


#ifndef TIM2_IRQ_PRIORITY
  #error TIM2_IRQ_PRIORITY not defined !
#endif

typedef void (*tb_callback_t)(void);
typedef void (*tb_capture_callback_t)(uint32_t t_us);

volatile tb_callback_t tb_at_fn = nullptr;
volatile tb_capture_callback_t tb_capture_fn = nullptr;


static inline uint32_t tb_now(void)
{
    return TIM2->CNT;
}

// a - b, signed; positive if a is later than b
static inline int32_t tb_diff(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b);
}


//-- debug pulses on the debug pin, ended by CH4

void dbg_pulse_start(uint16_t width_us)
{
    __disable_irq();
    gpio_high(DBG_PIN);
    TIM2->CCR4 = TIM2->CNT + width_us;
    LL_TIM_ClearFlag_CC4(TIM2);
    LL_TIM_EnableIT_CC4(TIM2);
    __enable_irq();
}


//-- one-shot scheduler on CH3

// Calls fn from the TIM2 interrupt at time t. Returns false, and schedules
// nothing, if t is less than TB_AT_MIN_LEAD_US away or already in the past.
// Only one call can be pending; a new tb_at() replaces the old one.
#define TB_AT_MIN_LEAD_US   20

bool tb_at(uint32_t t, tb_callback_t fn)
{
    __disable_irq();
    if (tb_diff(t, TIM2->CNT) < TB_AT_MIN_LEAD_US) {
        __enable_irq();
        return false;
    }
    tb_at_fn = fn;
    TIM2->CCR3 = t;
    LL_TIM_ClearFlag_CC3(TIM2);
    LL_TIM_EnableIT_CC3(TIM2);
    __enable_irq();
    return true;
}

void tb_at_cancel(void)
{
    LL_TIM_DisableIT_CC3(TIM2);
    LL_TIM_ClearFlag_CC3(TIM2);
    tb_at_fn = nullptr;
}

bool tb_at_pending(void)
{
    return LL_TIM_IsEnabledIT_CC3(TIM2);
}


//-- DIO1 capture

void tb_set_capture_callback(tb_capture_callback_t fn)
{
    tb_capture_fn = fn;
}


//-- interrupt

extern "C" { void TIM2_IRQHandler(void); }

void TIM2_IRQHandler(void)
{
#ifndef TDMA_DIO1_USE_EXTI
    if (LL_TIM_IsActiveFlag_CC1(TIM2)) { // DIO1 rose, CCR1 holds the time
        uint32_t t = TIM2->CCR1; // reading CCR1 clears CC1IF
        if (tb_capture_fn) tb_capture_fn(t);
    }
#endif
    if (LL_TIM_IsActiveFlag_CC3(TIM2) && LL_TIM_IsEnabledIT_CC3(TIM2)) {
        LL_TIM_ClearFlag_CC3(TIM2);
        LL_TIM_DisableIT_CC3(TIM2); // one-shot
        tb_callback_t fn = tb_at_fn;
        tb_at_fn = nullptr;
        if (fn) fn();
    }
    if (LL_TIM_IsActiveFlag_CC4(TIM2) && LL_TIM_IsEnabledIT_CC4(TIM2)) {
        LL_TIM_ClearFlag_CC4(TIM2);
        LL_TIM_DisableIT_CC4(TIM2);
        gpio_low(DBG_PIN);
    }
}


#ifdef TDMA_DIO1_USE_EXTI
extern "C" { void EXTI15_10_IRQHandler(void); }

void EXTI15_10_IRQHandler(void)
{
    uint32_t t = TIM2->CNT; // first thing, software latency is measured in task 2.5
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_15)) {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
        if (tb_capture_fn) tb_capture_fn(t);
    }
}
#endif


//-- init

uint32_t tb_timer_clock_hz(void)
{
    // TIM2 is on APB1; its clock is PCLK1, or 2 x PCLK1 if the APB1 prescaler is not 1
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    return (LL_RCC_GetAPB1Prescaler() == LL_RCC_APB1_DIV_1) ? pclk1 : 2 * pclk1;
}

// call after sx_init_gpio(), it takes PA15 over from the hal's GPIO setup
void tb_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_TIM2);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_TIM2);

    uint32_t tim_hz = tb_timer_clock_hz(); // 170 MHz on the mR900-30
    if (tim_hz % 1000000 != 0) while(1){} // must divide down to exactly 1 MHz

    LL_TIM_SetPrescaler(TIM2, tim_hz / 1000000 - 1);
    LL_TIM_SetAutoReload(TIM2, 0xFFFFFFFF);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(TIM2, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_GenerateEvent_UPDATE(TIM2); // load the prescaler now
    LL_TIM_ClearFlag_UPDATE(TIM2);

    // CH3, CH4: output compare, frozen (no pin), interrupt only
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_FROZEN);
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_FROZEN);

#ifndef TDMA_DIO1_USE_EXTI
    // CH1: input capture on TI1 = PA15, rising edge, no prescaler, no filter
    gpio_init_af(IO_PA15, IO_MODE_INPUT_PD, IO_AF_1, IO_SPEED_VERYFAST);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_ClearFlag_CC1(TIM2);
    LL_TIM_EnableIT_CC1(TIM2);
#else
    // DIO1 stays a GPIO input (sx_init_gpio); EXTI15 rising edge
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE15);
    LL_EXTI_DisableEvent_0_31(LL_EXTI_LINE_15);
    LL_EXTI_DisableFallingTrig_0_31(LL_EXTI_LINE_15);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_15);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_15);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_15);
    NVIC_SetPriority(EXTI15_10_IRQn, TIM2_IRQ_PRIORITY);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif

    NVIC_SetPriority(TIM2_IRQn, TIM2_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM2_IRQn);

    LL_TIM_SetCounter(TIM2, 0);
    LL_TIM_EnableCounter(TIM2);
}


#endif // TDMA_TIMEBASE_H
