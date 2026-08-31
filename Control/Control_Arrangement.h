#ifndef	__CONTROL_ARRANGEMENT_H
#define	__CONTROL_ARRANGEMENT_H

#include "stm32f10x.h"


void Control_Init(void);

void Control_Update(void);

void Control_SetMoveSpeed(float speed);

void Control_Motor_Update(void);

void Control_Reset(void);

#endif


