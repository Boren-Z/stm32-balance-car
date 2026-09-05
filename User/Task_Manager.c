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
/**
 * ================================================================
 * [调度层] 裸机多任务模型与状态机配置
 * ================================================================
 *  ├── 内部任务函数（static）
    │   ├── Task_Battery()
    │   ├── Task_Encoder()
    │   ├── Task_IMU()
    │   ├── Task_PID()
    │   └── Task_Debug()
    ├── 状态机（static，后续加）
    │   ├── current_state
    │   └── State_Machine_Handler()
    └── 对外接口
    ├── Task_Manager_Run()              ← 主循环唯一调用点
    └── Task_Manager_SetEvent()         ← 外部触发事件
 * ================================================================
 */





static void Task_Battery(void)
{
    static uint32_t last_battery = 0;  // 函数内部static，只属于这个任务
    // static uint16_t Raw_Voltage = 0;
    //判断要不要static的问题只有一个：这个变量的值需要在两次函数调用之间保留吗？
    uint32_t now = System_GetTick();

    if(now - last_battery >= 10)
    {
        last_battery = now;

        // if(Battery_GetFlag())
        // {
        //     Raw_Voltage = Battery_GetVoltageAnal();  // 更新电压值
        // }

        // // 电量显示逻辑放在flag外面，每10ms都执行
        // if      (Raw_Voltage > 3851) { LED_GPIO_SetBits(1, 1, 1); }
        // else if (Raw_Voltage > 3609) { LED_GPIO_SetBits(0, 1, 1); }
        // else if (Raw_Voltage > 3413) { LED_GPIO_SetBits(0, 0, 1); }
        // else if (Raw_Voltage > 3167) { LED_GPIO_SetBits(0, 0, 0); }
        // else
        // {
        //     static uint8_t blink_count = 0;
        //     blink_count++;
        //     if(blink_count <= 5)  LED_GPIO_SetBits(1, 1, 1);
        //     else                  LED_GPIO_SetBits(0, 0, 0);
        //     if(blink_count >= 10) blink_count = 0;
        // }

        float vbat = Battery_GetVoltage();  // 声明在这里

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

static void Task_Debug(void)
{
    // static uint32_t last_debug = 0;
    // uint32_t now = System_GetTick();
    // if(now - last_debug >= 500)
    // {
    //     last_debug = now;
    //     MPU6050_Info raw;
    //     MPU6050_GetData(&raw);
    //     char buf[80];
    //     sprintf(buf, "X:%d Y:%d Z:%d\r\n", raw.AccX, raw.AccY, raw.AccZ);
    //     Debugging_USART_SendString(buf);
    // }

    // static uint32_t last_debug = 0;
    // uint32_t now = System_GetTick();
    // if(now - last_debug >= 500)
    // {
    //     last_debug = now;
    //     IMU_Feedback_t imu;
    //     ComplementaryFilter_GetFeedBack(&imu);
    //     char buf[64];
    //     sprintf(buf, "ID:%02X pitch:%.1f\r\n", MPU6050_GetID(), imu.Pitch);
    //     Debugging_USART_SendString(buf);
    // }

    // static uint32_t last_debug = 0;
    // uint32_t now = System_GetTick();
    // if(now - last_debug >= 500)
    // {
    //     last_debug = now;
    //     MPU6050_Info raw;
    //     MPU6050_GetData(&raw);
    //     char buf[80];
    //     sprintf(buf, "X:%d Y:%d Z:%d\r\n", raw.AccX, raw.AccY, raw.AccZ);
    //     Debugging_USART_SendString(buf);
    // }

    // static uint32_t last_debug = 0;
    // uint32_t now = System_GetTick();
    // if(now - last_debug >= 2000)
    // {
    //     last_debug = now;
    //     IMU_Feedback_t imu;
    //     ComplementaryFilter_GetFeedBack(&imu);
    //     SpeedFeedback speed;
    //     EncoderFeedback_GetSpeed(&speed);
    //     // float vbat = Battery_GetVoltage();
    //     char buf[80];
    //     // sprintf(buf, "pitch:%.1f vbat:%.2f L:%.2f R:%.2f\r\n", imu.Pitch, vbat, speed.Left_Speed, speed.Right_Speed);
    //     Debugging_USART_SendString(buf);

    // static uint32_t last_debug = 0;
    // uint32_t now = System_GetTick();
    // if(now - last_debug >= 500)
    // {
    //     last_debug = now;
    //     float vbat = Battery_GetVoltage();
    //     uint16_t raw = Battery_GetVoltageAnal();
    //     char buf[64];
    //     sprintf(buf, "vbat:%.2f raw:%u\r\n", vbat, raw);
    //     Debugging_USART_SendString(buf);
    // }
    
    static uint32_t last_debug = 0;
    uint32_t now = System_GetTick();
    if(now - last_debug >= 500)
    {
        last_debug = now;
        IMU_Feedback_t imu;
        ComplementaryFilter_GetFeedBack(&imu);
        char buf[64];
        sprintf(buf, "pitch:%.1f\r\n", imu.Pitch);
        Debugging_USART_SendString(buf);
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




