#ifndef	__PID_H
#define	__PID_H
#include "stm32f10x.h"

typedef struct
{
    float KP;
    float KI;
    float KD;
    float DT;
    float Integral_UpperLimit;  // 积分限幅（anti-windup，算法内部）
    float Integral_LowerLimit;
    
/* 
    备选方案：输出限幅也放在这里
    float Output_UpperLimit;
    float Output_LowerLimit;
    → 优点：PID自包含，调用方不需要额外处理
    → 缺点：PID算法耦合了执行器的物理约束，不够纯粹
    → 当前选择：输出限幅由Control.c在调用PID_GetOutput()之后负责 
*/
} PID_Para_t;

typedef struct 
{
    /* data */
    // VAR：运行状态，每个周期更新
    float SP;             // VAR_INPUT：由外环通过PID_SetSP()写入

    float Error_Previous;
    float Error_Integral;
    float Error;
    
    float Output;

    uint8_t is_first;  // 第一次调用标志
} PID_State_t;


void  PID_Init(PID_Para_t* para, float kp, float ki, float kd, float upper, float lower, float dt);

void  PID_SetSP(PID_State_t* state, float sp);

void  PID_Update(PID_State_t* state, PID_Para_t* para, float FB);

float PID_GetOutput(PID_State_t* state);

void  PID_Reset(PID_State_t* state);


#endif

