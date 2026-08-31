#ifndef	__MOTOR_PWM_H
#define	__MOTOR_PWM_H
#include "stm32f10x.h"

void Double_Motors_Init(void);

void Motor_Speed_Set(int16_t Left_Speed, int16_t Right_Speed);
/*
 * TB6612FNG H桥控制真值表
 * ================================================================
 * IN1   IN2   PWM   STBY  OUT1  OUT2  模式
 * ----------------------------------------------------------------
 * H     H     H/L   H     L     L     Short brake（快速制动）
 * L     H     H     H     L     H     CCW（反转）
 * L     H     L     H     L     L     Short brake
 * H     L     H     H     H     L     CW（正转）
 * H     L     L     H     L     L     Short brake
 * L     L     H     H     OFF   OFF   Stop（自由滑行，高阻态）
 * H/L   H/L   H/L   L     OFF   OFF   Standby（待机）
 * ================================================================
 *
 * 平衡车使用策略：
 *   speed > 0 → IN1=H, IN2=L, PWM=占空比  → CW正转
 *   speed < 0 → IN1=L, IN2=H, PWM=占空比  → CCW反转
 *   speed = 0 → IN1=H, IN2=H, PWM=任意    → Short brake快速制动
 *
 * 注意：
 *   STBY必须保持H（PA1已在Motor_GPIO_Init中拉高）
 *   Stop（高阻态）不用于平衡车，因为需要快速制动响应
 * ================================================================
 */





#endif	//!__MOTOR_PWM_H
