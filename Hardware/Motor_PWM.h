#ifndef	__MOTOR_PWM_H
#define	__MOTOR_PWM_H
#include "stm32f10x.h"

/**
 * 【五层塔定位】驱动层 - 电机驱动模块
 *
 * 本模块在五层塔中的位置：
 *
 *   算法层    → 调用Motor_Speed_Set()输出PID控制量
 *   驱动层    ← 本模块，激活TIM1/TIM4 PWM + TB6612 GPIO
 *
 * 硬件架构：
 *   STM32 → TB6612FNG(H桥驱动) → 电机
 *
 *   左电机：
 *     MOTOR_L_PWM  → PA8  → TIM1_CH1（速度，复用推挽）
 *     MOTOR_L_IN1  → PA9  → GPIO（方向，普通推挽）
 *     MOTOR_L_IN2  → PA10 → GPIO（方向，普通推挽）
 *
 *   右电机：
 *     MOTOR_R_PWM  → PB6  → TIM4_CH1（速度，复用推挽）
 *     MOTOR_R_IN1  → PB5  → GPIO（方向，普通推挽）
 *     MOTOR_R_IN2  → PB7  → GPIO（方向，普通推挽）
 *
 *   TB6612使能：
 *     TB6612_STBY  → PA1  → GPIO（普通推挽，初始化后拉高）
 *
 * 对外接口（.h文件声明，外部只能看到这两个）：
 *   Double_Motors_Init()     → Action接口，初始化所有电机硬件
 *   Motor_Speed_Set()        → Set接口，算法层写入控制量
 *
 * 内部函数（static，外部不可见）：
 *   Motor_GPIO_Init()        → 方向控制引脚初始化
 *   Motor_Left_PWM_Init()    → 左电机PWM初始化（TIM1）
 *   Motor_Right_PWM_Init()   → 右电机PWM初始化（TIM4）
 *
 * 【两种初始化的区分】
 *   性质类初始化（Motor_GPIO_Init）→ 配置引脚模式，决定"能不能控制"
 *   数值类初始化（Motor_PWM_Init） → 配置PWM参数，决定"频率是多少"
 *   运行时更新（Motor_Speed_Set）  → 实时改变CCR和IN电平
 *
 * 【PWM频率选择10kHz的原因】
 *   太低(<1kHz)  → 电机嗡嗡响，电流波动大，效率低
 *   太高(>100kHz)→ TB6612开关损耗大，发热严重
 *   10kHz        → 高于人耳上限（20kHz边界），开关损耗可接受
 *                  控制周期5ms(200Hz)远低于PWM频率，电机响应足够快
 *
 * 【关于复用推挽输出】
 *   PWM引脚（PA8/PB6）→ 复用推挽（AF_PP）
 *   TIM1/TIM4外设控制引脚，CPU不参与，引脚控制权交给定时器
 *   方向引脚（PA9/PA10/PB5/PB7）→ 普通推挽（Out_PP）
 */



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
