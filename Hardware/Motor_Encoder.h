#ifndef	__MOTOR_ENCODER_H
#define	__MOTOR_ENCODER_H
#include "stm32f10x.h"

/**
 * ================================================================
 * 【五层塔定位】驱动层 - 编码器模块（T法测速）
 * ================================================================
 *
 * 本模块在五层塔中的位置：
 *
 *   算法层    → 调用Encoder_Get_L/R_Speed()获取实时转速反馈，
 *                未来接入闭环PID控制；Position可用于里程累计
 *   驱动层    ← 本模块，EXTI外部中断实现正交编码器测速测向
 *
 * 硬件架构：
 *   左编码器：ENC_L_A → PB14（EXTI中断触发相）
 *            ENC_L_B → PB15（被动读取相）
 *   右编码器：ENC_R_A → PB3 （EXTI中断触发相）
 *            ENC_R_B → PB4 （被动读取相）
 *
 *   PB3/PB4默认是JTAG调试引脚（TDO/NJTRST），必须先解除JTAG占用
 *   才能用作普通GPIO，同时保留SWD(PA13/14)给ST-Link调试不受影响
 *
 * 对外接口（.h文件声明，外部只能看到这五个）：
 *   Encoder_Init()            → Action接口，初始化GPIO/EXTI/NVIC
 *   Encoder_Get_L_Speed()     → Get interface, left wheel improved T-method speed (rad/s, auto-converges to 0 while decelerating)
 *   Encoder_Get_R_Speed()     → Get interface, right wheel improved T-method speed (same as above)
 *   Encoder_Get_L_Position()  → Get接口，左轮累计脉冲数
 *   Encoder_Get_R_Position()  → Get接口，右轮累计脉冲数
 *
 * 内部函数（static，外部不可见）：
 *   Encoder_GPIO_Init()  → 引脚模式配置+JTAG解除占用
 *   Encoder_EXTI_Init()  → A相双边沿中断配置
 *   Encoder_NVIC_Init()  → 中断优先级配置
 *
 * 【为什么放弃TIM硬件编码器模式，改用EXTI软件M法】
 *   STM32标准的TIM编码器模式需要A/B两相接在同一个定时器的CH1/CH2上
 *   查芯片复用功能表后发现两个编码器都不满足这个条件：
 *     PB14/PB15 → TIM1_CH2N/CH3N（互补通道，不是常规CH1/CH2）
 *     PB3/PB4   → TIM2_CH2/TIM3_CH1（分属两个不同定时器）
 *   两组引脚都无法凑出"同一定时器的CH1+CH2"，硬件编码器模式行不通
 *   改为EXTI外部中断+软件M法测速测向（与视频教程方案一致）
 *
 * 【M法测速的设计：为什么只有A相触发中断，B相只是被动读取】
 *   只在A相配置EXTI双边沿中断，B相是普通GPIO输入，
 *   在A相中断发生的瞬间"顺便看一眼"B相当前电平，用来判断方向
 *   这样每次脉冲只触发一次中断（而不是A、B两相各触发一次，
 *   避免重复计数和更复杂的双通道同步处理）
 * ================================================================
 */


void Encoder_Init(void);
float Encoder_Get_L_Speed(); // Get interface, left wheel improved T-method speed (rad/s, auto-converges to 0 while decelerating)
float Encoder_Get_R_Speed(); // right wheel 

int32_t Encoder_Get_L_Position(void);

int32_t Encoder_Get_R_Position(void);

#endif	//!__MOTOR_ENCODER_H
