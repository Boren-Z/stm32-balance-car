#ifndef	__COMPLEMENTARYFILTER_H
#define	__COMPLEMENTARYFILTER_H

#include "stm32f10x.h"
#include "I2C_MPU6050.h"


#define GYRO_SENSITIVITY    (2000.0f / 32768.0f)    // 单位：°/s per LSB
#define ACCEL_SENSITIVITY   (2.0f   / 32768.0f)     // 单位：g per LSB
#define ARLFA               0.95238f                  // 互补系数
#define COMPLEMENTARY_DT    0.005f                  // 互补滤波调用周期，单位秒

typedef struct
{
    float Pitch;     // 俯仰角，单位°
    float PitchDot;  // 俯仰角速度，单位rad/s

    float YawDot; 
} IMU_Feedback_t;


void ComplementaryFilter_Init(void);

void ComplementaryFilter_Update(void);

void ComplementaryFilter_GetFeedBack(IMU_Feedback_t* output);

float Control_GetOmegaRef(void);

#endif

