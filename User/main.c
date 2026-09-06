#include "stm32f10x.h"
#include "debugging_USART.h"
#include "ADC_Battery.h"
#include "GPIO_LED.h"
#include "SYSTEM_TIM.h"
#include "Task_Manager.h"
#include "Motor_PWM.h"
#include "Motor_Encoder.h"
#include "I2C_MPU6050.h"
#include "ComplementaryFilter.h"
#include "EncoderFeedback.h"
#include "Control_Arrangement.h"
#include <stdio.h>

/**
 *   - Main function
 * Based on the "Five-Layer + Defensive Programming" architecture theory
 *
 *   Five-layer tower architecture overview
 *   Monitoring layer      → low-voltage protection / tilt-angle emergency stop / watchdog / LED battery indicator
 *   Scheduling layer      → SysTick time base (1ms) / foreground-background task scheduling / flag mechanism
 *   Algorithm layer       → cascaded PID (angular velocity loop → angle loop → velocity loop)
 *   Feedback processing   → MPU6050 Euler-angle estimation / encoder M/T speed measurement / ADC conversion
 *   Driver layer          → GPIO/USART/TIM/ADC/I2C hardware activation and read/write
 *
 * Core principles of the theory:
 *
 * 1. Separation of Concerns
 *    Each layer does exactly one thing, communicating with adjacent
 *    layers only through interfaces, with no need to know what a
 *    non-adjacent layer is doing.
 *    → The driver layer doesn't know about PID, the algorithm layer doesn't know about registers
 *
 * 2. Requirements are derived top-down, constraints are reported bottom-up
 *    The monitoring layer defines "what's needed," the driver layer decides "what's possible"
 *    → Why SysTick: it doesn't compete with TIM1/TIM2/TIM4, which are
 *      already claimed by motor PWM and the ADC trigger — SysTick is a
 *      core Cortex-M peripheral, always available no matter how the
 *      general-purpose timers are allocated
 *
 * 3. Interfaces before implementation
 *    First define what each layer exposes externally (the .h file), then fill in the implementation (the .c file)
 *    → Data is hidden inside the .c file (static), interfaces are exposed in the .h file (function declarations)
 *
 * 4. Program to interfaces
 *    An upper layer calls interface functions, never touches a lower layer's internal variables directly
 *    → main.c only calls Battery_GetFlag(), never reads JEOC_Status directly
 *
 * Defensive programming principles:
 *
 * 1. Parameter validation at entry
 *    A function validates its parameters right at entry; illegal parameters report an error and return immediately
 *    → SendBytes: if(array==NULL || size==0) report_error(...)
 *
 * 3. The error-reporting function only depends on the lowest layer
 *    Prevents recursive calls from causing a stack overflow
 *    → report_error only uses SendByte, never SendString
 *
 * 4. Flag design
 *    An interrupt raises the flag, the main loop clears it automatically after consuming it, preventing duplicate handling
 *    → Battery_GetFlag() automatically clears JEOC_Status once it's read
 *
 * 5. Cache interface return values in a local variable
 *    Call the interface only once for the same piece of data, avoiding repeated calls and inconsistent data
 *    → uint16_t raw = Battery_GetVoltageAnal()
 *      then use raw for the comparisons, instead of calling Battery_GetVoltageAnal() multiple times
 *
 * Foreground-background scheduling model:
 *
 * Foreground (interrupts, urgent tasks):
 *   SysTick_Handler   → every 1ms, System_Tick++
 *   ADC1_2_IRQHandler → every 10ms, reads the ADC result, raises the flag
 *
 * Background (main loop, non-urgent tasks):
 *   Check Battery_GetFlag() → compare against thresholds, control the LEDs, print the voltage
 *   Check the tick interval → run each periodic task as needed
 *
 * Interrupt principle: the shorter the better — only gather data/raise the flag, business logic stays in the main loop
 *
 * Analogy to CODESYS:
 *   CODESYS Task (cyclic execution) ↔ main loop + tick check
 *   CODESYS Event Task              ↔ interrupt IRQHandler
 *   CODESYS global variables (GVL)  ↔ static volatile shared variables
 *   CODESYS FB encapsulation        ↔ .c/.h module encapsulation
 *   PLC scan-cycle auto-management  ↔ SysTick time base implemented by hand
 *
 * Methodology: from requirements to code
 *
 * Correct order:
 *   1. Read the schematic → build the peripheral map
 *   2. Determine the functional requirement → derive downward from the monitoring layer
 *   3. At each layer ask: what's needed? what are the constraints?
 *   4. Driver layer: configure the registers according to the peripheral map
 *   5. Acceptance: confirm the data using debug-UART prints
 *
 */
int main(void)
{
/* 
    * [1] NVIC priority grouping 
*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

/*
    * [2] LED GPIO initialization
    * Driver layer: PA4/PA5/PA6 push-pull output
    * Serves: the monitoring layer's battery-indicator logic
*/
    LED_GPIO_Init();

/*  * [3] Debug UART initialization
    * Driver layer: USART2, 115200 8N1
    * Spans every layer: every layer can output debug data over the UART
*/
    Debugging_USART_Init();
    Debugging_USART_SendString("step1: USART OK\r\n");

/*
    * [4] ADC trigger timer initialization
    * Driver layer: TIM2, 10ms period, TRGO triggers the ADC injected group
*/
    ADC_Battery_TIM_Init();
    Debugging_USART_SendString("step2: ADC_TIM OK\r\n");

/*
    * [5] ADC initialization
    * Driver layer: ADC1 injected group
*/
    ADC_Battery_Init();
    Debugging_USART_SendString("step3: ADC OK\r\n");

/*
    * [6] ADC interrupt controller initialization (preemption priority 2)
*/
    NVIC_Battery_Init();
    Debugging_USART_SendString("step4: NVIC_Battery OK\r\n");

/*
    * [7] System time base initialization
*/
    System_Init();
    Debugging_USART_SendString("step6: System OK\r\n");

/*
    * [9] Motor driver initialization
*/
    Double_Motors_Init();
    Debugging_USART_SendString("step7: Motors OK\r\n");

/*
    * [10] Encoder initialization
*/
    Encoder_Init();
    Debugging_USART_SendString("step8: Encoder OK\r\n");

/*
    * [11] MPU6050 initialization
*/
    I2C_MPU6050_Init();
    Debugging_USART_SendString("step9: MPU6050 OK\r\n");

/*
    * [12] Complementary filter initialization
*/
    ComplementaryFilter_Init();
    Debugging_USART_SendString("step10: Filter OK\r\n");

/*
    * [13] Control layer initialization
*/
    Control_Init();
    Debugging_USART_SendString("step11: Control OK\r\n");

/*
     * Main loop (background scheduler)
     * Scans through all tasks sequentially, each task decides for itself whether it should run
     * Not preemptive, a cooperative scheduler
*/
    /* [14] Motor enable/disable button, toggled by Control_Update() */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    while(1)
    {   // The scheduling layer's sole entry point, all tasks are scheduled here uniformly
        Task_Manager_Run();  
    }
}




