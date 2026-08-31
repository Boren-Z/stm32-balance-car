#ifndef	__I2C_MPU6050_H
#define	__I2C_MPU6050_H

/**
 * ================================================================
 * I2C/MPU6050姿态传感器驱动模块 —— 对外接口
 * ================================================================
 *
 * 五层塔定位：驱动层
 * 硬件：STM32 I2C1(重映射PB8/PB9) ←→ MPU6050(I2C地址0x68，AD0接GND)
 * 详细设计依据、踩坑记录见对应的.c文件
 * ================================================================
 */

#include "stm32f10x.h"

/* 六轴原始数据，单位：LSB（未转换为物理单位，转换在反馈处理层完成） */
typedef struct
{
    int16_t GyroX;
    int16_t GyroY;
    int16_t GyroZ;

    int16_t AccX;
    int16_t AccY;
    int16_t AccZ;
} MPU6050_Info;

/* 初始化I2C总线+MPU6050寄存器，调用一次即可 */
void I2C_MPU6050_Init(void);

/* 读WHO_AM_I寄存器，用于验证I2C通信是否正常建立 */
uint8_t MPU6050_GetID(void);

/* 读取六轴原始数据，结果写入Data指向的结构体 */
void MPU6050_GetData(MPU6050_Info* Data);

#endif	//!__I2C_MPU6050_H

