#include "stm32f10x.h"


static volatile uint16_t Voltage_Anal;   // Raw ADC Data, 0~4095
static volatile uint8_t  JEOC_Status;    // New Data Flag, 1 means new data arrived

static volatile float Vbat = 0.0f;


/* 
 * [Feedback processing layer interface] Get raw ADC value
 * Purpose: For monitoring-layer threshold checks — 
 * comparing raw ADC values directly is more efficient and avoids floating-point math
 * Threshold reference          (derived from hardware parameters, voltage-divider ratio = 3.3/8.4):
 *  3399 → Battery 70%~100%     (three LEDs on)
 *  3193 → Battery 40%~70%      (two LEDs on)
 *  2990 → Battery 10%~40%      (one LED on)
 * ≤2990 → Battery <10%         (all off, needs charging)
 * */

uint16_t Battery_GetVoltageAnal(void)
{
    return Voltage_Anal;
}

/* 
 * [Feedback processing layer interface] Get the converted actual voltage value
 *
 * Conversion formula derivation:
 *   V_bat = ADC value × (VREF+ / 4095) × (1 / divider ratio)
 *         = ADC value × (3.3 / 4095) × (8.4 / 3.3)
 *         = ADC value × (8.4 / 4095)
 * */

float Battery_GetVoltage(void)
{
    return Vbat + 1.4f;     //Adding the f suffix explicitly makes them float, which is faster.
}

/* 
 * [Scheduler layer interface] Check whether new ADC data is available
 * */
uint8_t Battery_GetFlag(void)
{
    uint8_t temp = JEOC_Status;
    JEOC_Status = 0;          // Cleared after being read, to prevent duplicate consumption.
    return temp;
}

/**
 * [Driver layer] ADC battery voltage acquisition - Timer initialization
 * Configure TIM2 as a 10ms periodic timer, triggering ADC injected-group sampling via TRGO hardware trigger.
 */
void ADC_Battery_TIM_Init(void)
{
    /* [1] Enable TIM2 clock
     * TIM2 is on the APB1 bus (low-speed, max 36MHz; 
     * after frequency multiplication, 72MHz is supplied to the timer) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* [2] Configure time-base parameters */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 72 - 1;               // 72MHz->1MHz, 1μs per count
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;         // keep default
    TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;   // Counting up, 0→ARR overflow
    TIM_TimeBaseInitStructure.TIM_Period        = 10000 - 1;            // 10000μs = 10ms overflow

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* [3] Configure the TRGO output source as the Update event
     * Each time the counter overflows (Update event), TRGO issues a trigger signal to the ADC*/
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

    /* [4] Enable TIM2, start counting */
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * [Driver layer] ADC battery voltage acquisition - ADC initialization
 *
 *  Signal flow:
 *   TIM2_TRGO(10ms) → ADC1 injected-group samples CH8(PB0)
 *   → Conversion complete → Result stored in JDR1 → Triggers JEOC interrupt
 *   → Interrupt reads JDR1 → Stores into Voltage_Anal → Raises JEOC_Status flag
 *   → Main loop sees the flag → Compares threshold → Controls LEDs
 *
 */
void ADC_Battery_Init(void)
{
    /* [1] Enable ADC1 and GPIOB clocks (both on the APB2 bus) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,  ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* [2] Configure PB0 as analog input*/
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* [3] Configure the injected-group channel (an injected-group-specific function, not part of ADC_InitTypeDef) */
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_1Cycles5);

    /* [4] Configure the injected-group external trigger source as TIM2_TRGO (an injected-group-specific function)
     * TIM2_TRGO is only in the injected-group trigger source list, 
     * not in the regular-group list (dictated by hardware constraints, fundemental reason) */
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T2_TRGO);

    /* [5] Configure basic ADC parameters (regular-group parameters, no effect on the injected group); 
     * the regular group does not use an external trigger */
    ADC_InitTypeDef ADC_InitStructure;
    // Single conversion, triggered by the timer, not automatically continuous
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                     
    // Right-aligned, result in the low 12 bits, read directly as 0~4095
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;         
    // Regular group does not use an external trigger
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;   
    // Independent mode, ADC1 operates independently
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;     
    // Regular group converts 1 channel
    ADC_InitStructure.ADC_NbrOfChannel       = 1; 
    // Single channel doesn't need scan mode                          
    ADC_InitStructure.ADC_ScanConvMode       = DISABLE;                     
    ADC_Init(ADC1, &ADC_InitStructure);                                     

    /* [6] Power on ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* [7] ADC self-calibration, eliminating internal capacitor error and improving conversion accuracy */
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1) == SET);

    /* [8] Enable the injected-group external trigger; 
     * only after enabling can the TIM2_TRGO signal actually trigger the ADC to start conversion */
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

    /* [9] Enable JEOC interrupt
     * Triggers an interrupt when the injected-group conversion completes; 
     * read the JDR1 result inside the IRQHandler */
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
}

/**
 * [Driver layer] NVIC interrupt controller configuration
 * This module's priority: preemption 2, subpriority 2 Lower than the TIM3 time base (preemption 0), 
 * ensuring the control cadence is never interrupted
 * ADC voltage sampling can afford to be a bit slower — occasional interruption has no impact
 */
void NVIC_Battery_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = ADC1_2_IRQn; // Shares the interrupt channel with ADC2
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;           // Lower than the TIM3 time base (0)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_Init(&NVIC_InitStructure);
}


/**
 * [Driver layer] ADC injected-group conversion-complete interrupt handler
 * 
 * Condition: ADC1 injected-group conversion complete (JEOC flag set)
**/
void ADC1_2_IRQHandler(void)
{
    if(ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == SET)
    {
        /* Read the raw ADC value */
        Voltage_Anal = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);

        /* Directly convert to a floating-point voltage (aligned with standard code) */
        Vbat = Voltage_Anal / 4095.0f * 8.4f;

        /* Raise the flag to notify */
        JEOC_Status = 1;

        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}


