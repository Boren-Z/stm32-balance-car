#ifndef	__MOTOR_ENCODER_H
#define	__MOTOR_ENCODER_H
#include "stm32f10x.h"

/**
 * [Five-Layer Tower Placement] Driver layer - Encoder module (T-method speed measurement)
 *
 * Position in the five layers:
 *
 *   Algorithm layer → Calls Encoder_Get_L/R_Speed() to get real-time speed
 *                      feedback, for future closed-loop PID control;
 *                      Position can be used for odometry accumulation
 *   Driver layer    ← This module, EXTI external interrupts implement
 *                      quadrature encoder speed and direction sensing
 *
 * Hardware architecture:
 *   Left encoder:  ENC_L_A → PB14 (EXTI interrupt-triggered phase)
 *                  ENC_L_B → PB15 (passively-read phase)
 *   Right encoder: ENC_R_A → PB3  (EXTI interrupt-triggered phase)
 *                  ENC_R_B → PB4  (passively-read phase)
 */



void Encoder_Init(void);
float Encoder_Get_L_Speed(void); // Get interface, left wheel improved T-method speed (rad/s, auto-converges to 0 while decelerating)
float Encoder_Get_R_Speed(void); // right wheel 

int64_t Encoder_Get_L_Position(void);

int64_t Encoder_Get_R_Position(void);

#endif	//!__MOTOR_ENCODER_H
