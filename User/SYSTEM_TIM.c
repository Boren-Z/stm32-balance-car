#include "stm32f10x.h"

/*
 * SysTick microsecond-level time base
 * TIM1/2/3/4 are all freed up for business logic to use
*/

/* VAR: internal state */
static volatile uint64_t System_Tick_Ms = 0;   // Millisecond counter
static float us_per_mini_tick = 0.0f;          // How many microseconds each SysTick count corresponds to

/**
 * [Scheduling layer interface] Get the current system timestamp
 */
uint32_t System_GetTick(void)
{
    return (uint32_t)System_Tick_Ms;
}

/**
 * [Scheduling layer] SysTick initialization
 */
void System_Init(void)
{
    RCC_ClocksTypeDef clockinfo = {0};
    RCC_GetClocksFreq(&clockinfo);

    uint32_t load = clockinfo.HCLK_Frequency / 1000 - 1;  // 1ms period
    SysTick->LOAD = load;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk    // AHB clock source
                  | SysTick_CTRL_TICKINT_Msk      // Enable interrupt
                  | SysTick_CTRL_ENABLE_Msk;      // Enable counting

    SCB->SHP[7] = 0;  // SysTick has the highest priority

    us_per_mini_tick = 1000.0f / (load + 1);
}

/**
 * [Scheduling layer] SysTick interrupt handler
 */
void System_Tick_Update(void)
{
    System_Tick_Ms++;
}

/**
 * [Scheduling layer] Get the microsecond timestamp
 *
 * Principle: the millisecond part × 1000 + the SysTick counter's remainder converted to microseconds
 * Atomic protection: interrupts are disabled while reading, to prevent tick and VAL from becoming inconsistent
 */
uint64_t System_GetUs(void)
{
    uint64_t tick;
    uint32_t mini_tick;

    __disable_irq();
    while(1)
    {
        tick = System_Tick_Ms;
        mini_tick = SysTick->VAL;
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG)
            System_Tick_Ms++;
        else
            break;
    }
    __enable_irq();

    tick *= 1000;
    tick += (uint32_t)((SysTick->LOAD - mini_tick) * us_per_mini_tick);
    return tick;
}


