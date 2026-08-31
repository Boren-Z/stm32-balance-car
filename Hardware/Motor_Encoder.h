#ifndef	__MOTOR_ENCODER_H
#define	__MOTOR_ENCODER_H
#include "stm32f10x.h"

void Encoder_Init(void);

// int32_t Encoder_Get_L_Speed(void);

// int32_t Encoder_Get_R_Speed(void);

float   Encoder_Get_L_Speed(void);
float   Encoder_Get_R_Speed(void);

int32_t Encoder_Get_L_Position(void);

int32_t Encoder_Get_R_Position(void);


#endif	//!__MOTOR_ENCODER_H
