#ifndef	__COMPLEMENTARYFILTER_H
#define	__COMPLEMENTARYFILTER_H

#include "stm32f10x.h"
#include "I2C_MPU6050.h"


/**
 * [Five-Layer Placement] Feedback processing layer - Complementary filter attitude estimation module
 *
 * This module's position in the five layers:
 *
 *   Algorithm layer → Calls ComplementaryFilter_GetPitch() to get the pitch angle
 *   Feedback layer  ← This module, fuses raw MPU6050 data into a usable attitude angle
 *   Driver layer    → This module calls I2C_MPU6050's MPU6050_GetData() to get raw data
 *
 * Physical coordinate system (confirmed from the actual mounting orientation):
 *   Y-axis → direction the vehicle faces (direction of travel)
 *   X-axis → direction of the wheel axle (left/right)
 *   Z-axis → perpendicular to the ground (up/down)
 *
 *   pitch = rotation about the X-axis → the vehicle tipping forward/backward, the angle the balance car needs to control
 *   roll  = rotation about the Y-axis → the vehicle body leaning side to side, used for safety-layer monitoring
 *   yaw   = rotation about the Z-axis → for future Bluetooth steering, obtained purely from GyroZ integration, no complementary filtering needed
 *
 *
 * [Core math behind the complementary filter]
 *   Two paths reach the same destination by different routes — both can
 *   compute pitch, but each has its own flaw:
 *
 *   Gyroscope path (integration method):
 *     angle = ∫angular velocity dt (a single integration, not angular acceleration)
 *     Pro: smooth and accurate over short timescales
 *     Con: inherent bias gets amplified by accumulation over the integral, causing long-term drift (same effect as an oversized PID integral term)
 *
 *   Accelerometer path (trigonometric method):
 *     angle = atan2(AccY, AccZ) (no integration, computed independently each time)
 *     Pro: stable long-term, no drift
 *     Con: instantaneous readings are easily disturbed by motion/vibration, with large high-frequency jitter
 *
 *   Complementary filter fusion formula:
 *     pitch = α×(pitch_prev + gyro×dt) + (1-α)×accel_pitch
 *     α close to 1 (e.g. 0.98) → trust the gyroscope short-term, rely on the accelerometer long-term to correct drift
 *
 *   Viewed in the frequency domain (Laplace transform):
 *     Gyroscope part = high-pass filter H_gyro(s) = s/(s+1/τ)
 *     Accelerometer part = low-pass filter H_accel(s) = (1/τ)/(s+1/τ)
 *     H_gyro(s) + H_accel(s) = 1 (complementary, covers the full frequency range, no blind spots)
 *     Analogous to a speaker crossover: the woofer + tweeter together reproduce the complete sound
 *
 * [The design decision behind dt]
 *   dt is fixed at 0.005f (a 5ms constant), not measured dynamically
 *   Reason: the PERIODIC(5) macro guarantees Update() is called exactly every 5ms, so dt is a trustworthy fixed value
 *
 * [Why only pitch is computed, not roll]
 *   pitch → the motors can actively correct it, must be controlled in real time
 *   roll  → the two wheels share the same axle, the motors physically cannot correct it; the computed value is only used by the safety layer, lower priority
 */

#define GYRO_SENSITIVITY    (2000.0f / 32768.0f)    // Unit: °/s per LSB
#define ACCEL_SENSITIVITY   (2.0f   / 32768.0f)     // Unit: g per LSB
#define ARLFA               0.95238f                // Complementary filter coefficient
#define COMPLEMENTARY_DT    0.005f                  // Complementary filter call period, in seconds

typedef struct
{
    float Pitch;     // Pitch angle, in degrees
    float PitchDot;  // Pitch angular velocity, in rad/s
    float YawDot;
} IMU_Feedback_t;


void ComplementaryFilter_Init(void);

void ComplementaryFilter_Update(void);

void ComplementaryFilter_GetFeedBack(IMU_Feedback_t* output);

float Control_GetOmegaRef(void);

#endif

