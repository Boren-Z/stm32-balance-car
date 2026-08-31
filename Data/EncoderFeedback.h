#ifndef	__ENCODERFEEDBACK_H
#define	__ENCODERFEEDBACK_H

#include "stm32f10x.h"
#include "Motor_Encoder.h"

#define ENCODER_LINE_COUNTE                         22
#define REDUCTION_RATIO                             (30613.0f / 1500.0f)
#define TOTAL_NUM_OF_PULSES_PER_WHEEL_REVOLUTION    449
#define FEEDBACK_DT                                 0.005f                // 调用周期，单位秒


typedef struct
{
    /* data */
    float Left_Speed;
    float Right_Speed;
} SpeedFeedback;



void EncoderFeedback_Update(void);   // 每5ms调用，读脉冲增量，换算成轮速



void EncoderFeedback_GetSpeed(SpeedFeedback* Outputs); // 返回平均轮速（m/s）



#endif

