#ifndef	__ADC_BATTERY_H
#define	__ADC_BATTERY_H
#include "stm32f10x.h"

/**
 * ================================================================
 * 【五层塔定位】驱动层 - ADC电池电压采集模块
 * ================================================================
 *
 * 本模块在五层中的应用：
 *
 *   监控层     → 由外部调用Battery_GetVoltageAnal()比较阈值，控制LED
 *   调度层     → Battery_GetFlag()驱动主循环任务调度
 *   算法层     → Battery_GetVoltage()完成ADC值→电压的线性换算
 *   反馈处理层 → 分压比校正（R6/R14），VREF+基准确认
 *   驱动层     ← 本模块，激活TIM2/ADC1硬件，产生JEOC中断
 *
 * 对外接口（.h文件声明，外部只能通过这些函数访问）：
 *   Battery_GetVoltageAnal() → 获取原始ADC值（用于阈值判断）
 *   Battery_GetVoltage()     → 获取换算后电压（用于调试打印）
 *   Battery_GetFlag()        → 查询是否有新数据（消费后自动清零）
 *   ADC_Battery_TIM_Init()   → TIM2初始化
 *   ADC_Battery_Init()       → ADC1初始化
 *   NVIC_Battery_Init()      → 中断控制器初始化
 *
 * 【防御性编程原则】
 *   1. static  → 内部变量不对外暴露，外部只能通过接口函数访问
 *   2. volatile → 中断和主循环共享的变量必须加volatile，
 *                 防止编译器优化导致主循环读到缓存旧值
 *   3. flag机制 → 中断只举旗，主循环负责处理，职责分离
 *   4. GetFlag()读完自动清零 → 防止同一次数据被重复处理
 * ================================================================
 */

uint16_t Battery_GetVoltageAnal(void);

float Battery_GetVoltage(void);

uint8_t Battery_GetFlag(void);

void ADC_Battery_TIM_Init(void);

void ADC_Battery_Init(void);

void NVIC_Battery_Init(void);

#endif //!__ADC_BATTERY_H

