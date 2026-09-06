#include "stm32f10x.h"
#include "Task_Manager.h"
#include "SYSTEM_TIM.h"
#include "ADC_Battery.h"
#include "GPIO_LED.h"
#include "ComplementaryFilter.h"
#include "EncoderFeedback.h"
#include "Control_Arrangement.h"
#include "debugging_USART.h"
#include "I2C_MPU6050.h"
#include <stdio.h>
#include <stdarg.h>



/** [Scheduling layer] Bare-metal multi-task model and state machine configuration */

static void Task_Battery(void)
{
    static uint32_t last_battery = 0;

    uint32_t now = System_GetTick();

    if(now - last_battery >= 10)
    {
        last_battery = now;

        float vbat = Battery_GetVoltage();  // Declared here

        if      (vbat > 7.9f) { LED_GPIO_SetBits(1, 1, 1); }
        else if (vbat > 7.4f) { LED_GPIO_SetBits(0, 1, 1); }
        else if (vbat > 7.0f) { LED_GPIO_SetBits(0, 0, 1); }
        else if (vbat > 6.5f) { LED_GPIO_SetBits(0, 0, 0); }
        else
        {
            static uint8_t blink_count = 0;
            blink_count++;
            if(blink_count <= 5) LED_GPIO_SetBits(1, 1, 1);
            else                 LED_GPIO_SetBits(0, 0, 0);
            if(blink_count >= 10) blink_count = 0;
        }

    }
}

static void Task_IMU(void)
{
    static uint32_t last_imu = 0;
    uint32_t now = System_GetTick();
    if(now - last_imu >= 5)
    {
        last_imu = now;
        ComplementaryFilter_Update();
        EncoderFeedback_Update();
    }
}

static void Task_Control(void)
{
    static uint32_t last_control = 0;
    uint32_t now = System_GetTick();
    if(now - last_control >= 5)
    {
        last_control = now;
        Control_Update();
    }
}

/**
 * [Scheduling layer - internal] Periodic-timer check (static helper)
 *
 * Encapsulates the "static last_tick; now; if(now-last_tick>=period)"
 * pattern that used to be hand-copied into every debug print block.
 * Each call site owns its own *last_tick (declared static at the call
 * site), so multiple independent periods can coexist safely.
 */
static uint8_t Debug_Every(uint32_t* last_tick, uint32_t period_ms)
{
    uint32_t now = System_GetTick();
    if(now - *last_tick >= period_ms)
    {
        *last_tick = now;
        return 1;
    }
    return 0;
}

/**
 * [Scheduling layer - internal] Formatted debug print (static helper)
 *
 * Wraps sprintf + Debugging_USART_SendString into one call, so a new
 * debug print no longer needs its own "char buf[N]; sprintf(...);
 * Debugging_USART_SendString(buf);" boilerplate.
 */
static void Debug_Printf(const char* fmt, ...)
{
    char buf[80];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Debugging_USART_SendString(buf);
}

static void Task_Debug(void)
{
    static uint32_t last_debug = 0;
    if(Debug_Every(&last_debug, 500))
    {
        IMU_Feedback_t imu;
        ComplementaryFilter_GetFeedBack(&imu);
        Debug_Printf("pitch:%.1f\r\n", imu.Pitch);
    }
}

static void Task_Motor(void)
{
    static uint32_t last_motor = 0;
    uint32_t now = System_GetTick();
    if(now - last_motor >= 1)
    {
        last_motor = now;
        Control_Motor_Update();
    }
}



void Task_Manager_Run(void)
{
    Task_Motor();
    Task_Battery();
    Task_IMU();
    Task_Control();
    Task_Debug();
}




