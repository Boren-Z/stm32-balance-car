#ifndef __SYSTEM_TIM_H
#define __SYSTEM_TIM_H
#include "stm32f10x.h"

/* ============================================================
 * 旧方案：TIM3毫秒级时基（注释保留备用）
 * void TIM_System_Schedule(void);
 * void NVIC_System_Init(void);
 * ============================================================ */

/* ============================================================
 * 新方案：SysTick微秒级时基（对齐标准代码）
 * ============================================================ */
void     System_Init(void);        // 替代TIM_System_Schedule()+NVIC_System_Init()
uint32_t System_GetTick(void);     // 保持不变，毫秒级，兼容所有调用方
uint64_t System_GetUs(void);       // 新增，微秒级，供PID动态deltaT使用

#endif  //!__SYSTEM_TIM_H

