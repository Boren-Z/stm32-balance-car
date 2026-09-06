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
 *
 * Architecture: 5-layer "OLA" tower + defensive programming.
 * See README.md for the full architecture breakdown, design principles,
 * and the foreground/background scheduling model.
 *
 * Five-layer overview (top to bottom):
 *   Monitoring → Scheduling → Algorithm → Feedback Processing → Driver
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




