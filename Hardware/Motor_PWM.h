#ifndef	__MOTOR_PWM_H
#define	__MOTOR_PWM_H
#include "stm32f10x.h"

/**
 * [Five-Layer Placement] Driver layer - Motor driver module
 * 
 * This module's position in the five layers:
 * Algorithm layer → Calls Motor_Speed_Set() to output the PID control quantity
 * Driver layer ← This module, activates TIM1/TIM4 PWM + TB6612 GPIO
 * 
 * Hardware architecture:
 * STM32 → TB6612FNG (H-bridge driver) → Motor
 * 
 * Left motor:
 * MOTOR_L_PWM  → PA8  → TIM1_CH1 (speed, alternate-function push-pull)
 * MOTOR_L_IN1  → PA9  → GPIO (direction, normal push-pull)
 * MOTOR_L_IN2  → PA10 → GPIO (direction, normal push-pull)
 * 
 * Right motor:
 * MOTOR_R_PWM  → PB6  → TIM4_CH1 (speed, alternate-function push-pull)
 * MOTOR_R_IN1  → PB5  → GPIO (direction, normal push-pull)
 * MOTOR_R_IN2  → PB7  → GPIO (direction, normal push-pull)
 * 
 * TB6612 enable:
 * TB6612_STBY  → PA1  → GPIO (normal push-pull, pulled high after initialization)
*/



void Double_Motors_Init(void);

void Motor_Speed_Set(int16_t Left_Speed, int16_t Right_Speed);
/* TB6612FNG H-bridge
 *
 * IN1 IN2  PWM   STBY OUT1  OUT2 Mode
 * H    H    H/L   H    L     L   Short brake (fast braking)
 * L    H    H     H    L     H   CCW (reverse)
 * L    H    L     H    L     L   Short brake
 * H    L    H     H    H     L   CW (forward)
 * H    L    L     H    L     L   Short brake
 * L    L    H     H   OFF   OFF  Stop (free-wheeling, high-impedance)
 * H/L  H/L  H/L   L   OFF   OFF  Standby
 * 
 * STBY must be kept high (PA1 is already pulled high in Motor_GPIO_Init)
 * Stop (high-impedance) is not used for the balance car, since fast brake response is needed
 * 
*/





#endif	//!__MOTOR_PWM_H
