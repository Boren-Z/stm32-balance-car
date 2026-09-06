#include "stm32f10x.h"
#include "ComplementaryFilter.h"
#include "I2C_MPU6050.h"
#include <math.h>

/* 
 * VAR_INPUT (read internally via MPU6050_GetData(), not exposed externally):
 *   Raw_Data.GyroX  → Gyroscope X-axis raw LSB value (angular velocity about the X axis, corresponds to pitch change)
 *   Raw_Data.AccY   → Accelerometer Y-axis raw LSB value (gravity component along Y)
 *   Raw_Data.AccZ   → Accelerometer Z-axis raw LSB value (gravity component along Z)
 *
 * VAR_OUTPUT (exposed externally via GetPitch()):
 *   Pitch           → Current pitch angle, in degrees; positive = leaning forward, negative = leaning backward
 *
 * VAR (internal state, persists across calls):
 *   Raw_Data        → Raw sensor data cache (refreshed on every Update())
 *   pitch_prev      → Previous angle value from the gyroscope integration path (inside Filter_Gyro)
 **/

static MPU6050_Info Raw_Data;   

static IMU_Feedback_t Output;


/**
 * [Feedback layer] Accelerometer path: gravity component → pitch candidate value (static)
 *
 * Principle: when the vehicle body tilts, the ratio of the gravity
 *            component on AccY/AccZ changes, so atan2() is used to
 *            back-calculate the tilt angle
 *            This is an independent instantaneous value computed each
 *            time, doesn't depend on history, and won't drift
 */
static float Filter_ACCL(void)
{
    float accY = Raw_Data.AccY * ACCEL_SENSITIVITY;
    float accZ = Raw_Data.AccZ * ACCEL_SENSITIVITY;
    return atan2f(accY, accZ) * (180.0f / 3.1415926f);
}

/**
 * [Feedback layer] Gyroscope path: angular velocity integration → pitch candidate value
 *
 * Principle: pitch = pitch_prev + GyroX × conversion factor × dt
 *            Each call integrates the angular velocity by one step,
 *            accumulating onto the previous angle value
 */
static float Filter_Gyro(void)
{
    float gyroX = Raw_Data.GyroX * GYRO_SENSITIVITY;
    // Integrate starting from the pitch after the last filter pass, not from a value the gyroscope integrated on its own
    float pitch_gyro = Output.Pitch + gyroX * COMPLEMENTARY_DT;
    return pitch_gyro;
}

/**
 * [Feedback layer] Complementary filter fusion (static)
 *
 * Formula: pitch = α×gyroscope candidate value + (1-α)×accelerometer candidate value
 * α=0.98: trust the gyroscope in the short term (fast high-frequency
 *         response), rely on the accelerometer long-term to correct
 *         drift (stable at low frequency)
 */
static float Filter_Update(void)
{
    return ARLFA * Filter_Gyro() + (1 - ARLFA) * Filter_ACCL();
}

/* [Feedback layer] Complementary filter initialization (external Action interface) */
void ComplementaryFilter_Init(void)
{
    Output.Pitch = 0;
    Output.PitchDot = 0;
    Output.YawDot = 0;
}

/**
 * [Feedback layer] Complementary filter update (external Update interface, called by Task_Manager every 5ms)
 *
 * Responsibility: read the latest raw data first, then trigger the computation, updating Pitch
 *
 * [Design principle of the Update interface]
 *   Update = "go do a computation now, advancing the state by one step"
 *   Called periodically by the scheduling layer, has timing requirements
 */
void ComplementaryFilter_Update(void)
{
    MPU6050_GetData(&Raw_Data);
/*   Update() proactively calls MPU6050_GetData() internally
 *   The algorithm layer doesn't need to know what MPU6050_Info is at all,
 *   keeping the interface cleaner
 *   This is exactly the five-layer-tower principle in action: "data flows
 *   within a layer, layers only exchange results with each other"
 */
    Output.Pitch    = Filter_Update();
    Output.PitchDot = Raw_Data.GyroX * GYRO_SENSITIVITY * (3.1415926f / 180.0f);
    Output.YawDot   = Raw_Data.GyroZ * GYRO_SENSITIVITY * (3.1415926f / 180.0f); 
}

/**
 * [Feedback layer] Get the current pitch angle (external Get interface)
 *
 * Responsibility: only reads the already-computed Pitch value, doesn't trigger any computation
 *
 * [Two forms of Get interfaces]
 *   Return form (single value): float ComplementaryFilter_GetPitch(void)
 *   Pointer form (struct): void EncoderFeedback_GetSpeed(SpeedFeedback* Output)
 *   Analogous to CODESYS: reading an FB's VAR_OUTPUT without triggering FB execution
 */
void ComplementaryFilter_GetFeedBack(IMU_Feedback_t* output)
{
    *output = Output;
}
