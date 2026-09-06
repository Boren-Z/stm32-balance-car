#include "stm32f10x.h"
#include "Control_Arrangement.h"
#include "ADC_Battery.h"  // Needed for Battery_GetVoltage()
#include "PID.h"
#include "ComplementaryFilter.h"
#include "EncoderFeedback.h"
#include "SYSTEM_TIM.h"
#include "Motor_PWM.h"
#include <math.h>
#include <stdio.h>
#include "debugging_USART.h"
#include "Motor_Encoder.h"

/* ============================================================
 * VAR_INPUT (read via the feedback layer's Get interfaces):
 *   Speed_FB.Left_Speed          → left wheel speed, from EncoderFeedback
 *   Speed_FB.Right_Speed         → right wheel speed, from EncoderFeedback
 *   Imu_FB.Pitch                 → actual tilt angle, from ComplementaryFilter
 *   Imu_FB.PitchDot              → actual angular velocity, from ComplementaryFilter
 *
 * VAR (internal state, static):
 *   Para_Velocity/State_Velocity → velocity loop PID instance
 *   Para_Theta/State_Theta       → angle loop PID instance
 *   Para_ThetaDot/State_ThetaDot → angular velocity loop PID instance
 *   Last_Velocity                → velocity loop timer
 *
 * VAR_OUTPUT:
 *   None (output is written directly to the driver layer via Motor_Speed_Set(), not exposed externally)
 * ============================================================ */
extern float omega_ref;

static PID_Para_t  Para_Velocity;
static PID_State_t State_Velocity;

static PID_Para_t  Para_Theta;
static PID_State_t State_Theta;

static PID_Para_t  Para_ThetaDot;
static PID_State_t State_ThetaDot;

static PID_Para_t  Para_MotorOmega_L;
static PID_State_t State_MotorOmega_L;

static PID_Para_t  Para_MotorOmega_R;
static PID_State_t State_MotorOmega_R;

static PID_Para_t  Para_Turn;
static PID_State_t State_Turn;
static float omega_ref = 0.0f;


float Control_GetOmegaRef(void)
{
    return omega_ref;
}


/**
 * [Algorithm layer] Control initialization (external Action interface)
 *
 *
 *   Integral clamping limits:
 *   Velocity loop: ±0.5g (about ±4.9rad), corresponds to a reasonable target-tilt-angle range
 *   Angle loop: ±4π rad/s (about ±12.57), corresponds to a reasonable target-angular-velocity range
 *   Angular velocity loop: ±40π rad/s² (about ±125.7), corresponds to a reasonable angular-acceleration range
 */
void Control_Init(void)
{
    // Velocity loop (outermost loop, 5ms)
    PID_Init(&Para_Velocity, 7.0f, 2.0f, 0.0f, 0.5f*9.8f, -0.5f*9.8f, 0.005f);

    // Angle loop (middle loop, 5ms)
    PID_Init(&Para_Theta,     8.0f,  0.0f,  0.0f, 12.57f,    -12.57f,    0.005f);

    // Angular velocity loop (innermost loop, 5ms)
    PID_Init(&Para_ThetaDot,  10.0f, 10.0f, 0.0f, 125.7f,    -125.7f,    0.005f);

    // Motor speed loop (4th loop, 1ms)
    PID_Init(&Para_MotorOmega_L, 0.5f, 3.0f, 0.0f, 8.4f, -8.4f, 0.001f);
    PID_Init(&Para_MotorOmega_R, 0.5f, 3.0f, 0.0f, 8.4f, -8.4f, 0.001f);

    PID_Init(&Para_Turn,      1.0f,  0.0f,  0.0f, 15.0f, -15.0f, 0.005f);

}

/*
    * [Algorithm layer] Control update (external Update interface, called by the scheduling layer every 5ms)
    *
    * Cascade execution order:
    *   1. Read all feedback quantities from the feedback layer
    *   2. Velocity loop (50ms): computes the target tilt angle → SetSP for the angle loop
    *   3. Angle loop (5ms): computes the target angular velocity → SetSP for the angular velocity loop
    *   4. Angular velocity loop (5ms): computes the final control quantity → Motor_Speed_Set()
*/
void Control_Update(void)
{

    /* Button enable (PA11, active low). Motors are off by default at power-on; one button press starts control */
    static uint8_t motor_enabled = 0;
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET)
    {
        motor_enabled = !motor_enabled;
        Control_Reset();
        // Debounce
        while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11) == Bit_RESET);
    }

    if(motor_enabled == 0)
    {
        Motor_Speed_Set(0, 0);
        return;
    }

/*
    * Step 1: Read all feedback quantities from the feedback layer. Strictly follows the five-layer-tower principle: the algorithm layer only calls feedback-layer interfaces, never reaches across to the driver layer
*/
    SpeedFeedback  Speed_FB;
    IMU_Feedback_t Imu_FB;
    EncoderFeedback_GetSpeed(&Speed_FB);
    ComplementaryFilter_GetFeedBack(&Imu_FB);

/*
    * Step 2: Tilt-angle safety protection. Stop immediately if the tilt angle exceeds 45°, preventing the motors from running at full speed while the robot is falling
*/

    float theta     = Imu_FB.Pitch * (3.1415926f / 180.0f);
    float theta_dot = Imu_FB.PitchDot;

/*
    * Step 3: Speed correction — remove the "false wheel speed" produced by tilting.
    * When the balance car's body tilts, even without any real forward
    * translation, the motion of the center of mass also drives the wheels
    * to turn — this portion of rotation is "forced" and doesn't mean the
    * car is actually moving forward.
    * If this isn't removed, the velocity loop will misjudge the actual
    * speed and make the wrong correction.
*/
    static const float g  = 9.8f;
    static const float lp = 0.062f;                 // Distance from wheel axle to center of mass (0.062m)
    static const float rw = 0.032f;                 // Wheel radius (0.032m)

    /* False wheel speed produced by the center of mass rotating about the wheel axle: */
    float omega  = 0.5f * (Speed_FB.Left_Speed + Speed_FB.Right_Speed);
    float omega2 = -theta_dot * (lp + rw) / rw;    // -theta_dot: the tilt direction is opposite to the wheel-speed direction
    float omega1 = omega - omega2;                 // True wheel speed = measured wheel speed - false wheel speed
    float x_dot  = omega1 * rw;                    // True linear speed = true wheel speed × wheel radius

/*
    * Step 4: Velocity loop
    * Conversion from velocity-loop output to target tilt angle:
    * Newton's second law: F = ma, horizontal force = mg×tan(theta)
    * Therefore: a = g×tan(theta) → theta = atan(a/g)
*/
    static uint32_t Last_Velocity = 0;
    uint32_t now = System_GetTick();

    if(now - Last_Velocity >= 5)
    {
        Last_Velocity = now;
        PID_Update(&State_Velocity, &Para_Velocity, x_dot);
        float theta_ref = atanf(PID_GetOutput(&State_Velocity) / g);
        PID_SetSP(&State_Theta, theta_ref);
    }

/*
    * Step 5: Angle loop
    * FB = theta (actual tilt angle, radians)
    * SP = theta_ref (target tilt angle output by the velocity loop, radians)
    * Output = theta_dot_ref (target angular velocity, rad/s)
*/
    PID_Update(&State_Theta, &Para_Theta, theta);
    float theta_dot_ref = PID_GetOutput(&State_Theta);
    PID_SetSP(&State_ThetaDot, theta_dot_ref);

/*
    * Step 6: Angular velocity loop
    * FB = theta_dot (actual angular velocity, rad/s)
    * SP = theta_dot_ref (target angular velocity output by the angle loop, rad/s)
    * Output = theta_dot_dot_ref (target angular acceleration, rad/s²)
*/
    PID_Update(&State_ThetaDot, &Para_ThetaDot, theta_dot);
    float theta_dot_dot_ref = PID_GetOutput(&State_ThetaDot);

/*
    * Step 7: Inverse solve → wheel speed → PWM
*/
    // Inverted-pendulum equation of motion:
    float x_dot_dot_ref = (g * sinf(theta) - theta_dot_dot_ref * lp) / cosf(theta);

    static uint32_t last_time = 0;
    float dt = (now - last_time) * 0.001f;
    if(last_time != 0)
    {   // Integrating linear acceleration gives wheel speed:
        omega_ref += (1.0f / rw) * x_dot_dot_ref * dt;
    }
    last_time = now;

    // Clamp omega_ref — prevents the integrator from running away; ±10 rad/s corresponds to PWM ±100, a physically reasonable range
    if(omega_ref >  7.0f) omega_ref =  7.0f;
    if(omega_ref < -7.0f) omega_ref = -7.0f;

    // Turn loop: reads yaw angular velocity, computes the differential correction amount
    PID_Update(&State_Turn, &Para_Turn, Imu_FB.YawDot);
    float omega_diff = PID_GetOutput(&State_Turn);

    // Left/right wheel differential output (turn-loop correction)
    PID_SetSP(&State_MotorOmega_L, omega_ref + omega_diff);
    PID_SetSP(&State_MotorOmega_R, omega_ref - omega_diff);
}



/**
 * [Algorithm layer] Motor speed closed loop (4th loop, external Update interface, called by Task_Manager every 1ms)
 *
 *   The three-loop control logic outputs omega_ref (target wheel speed, rad/s)
 *   But Motor_Speed_Set() needs a PWM duty cycle (-100~100)
 *   Directly using omega_ref × a coefficient is open-loop; changes in
 *   battery voltage would cause:
 *     the same PWM producing a faster speed on a full battery, and a
 *     slower speed on a nearly depleted one
 *     control effectiveness drifting with battery voltage, unable to balance stably
 *
 * [How the 4th loop works]
 *   SP = omega_ref (target wheel speed output by the three-loop cascade)
 *   FB = actual wheel speed (measured by the encoder)
 *   PID output = voltage Ua (in V)
 *   PWM duty cycle = Ua / Vbat × 100% → dividing by battery voltage compensates for voltage changes, keeping speed stable
 */
void Control_Motor_Update(void)
{
    // Read the actual wheel speed (feedback-layer interface)
    SpeedFeedback Speed_FB;
    EncoderFeedback_GetSpeed(&Speed_FB);

    // 4th-loop PID computation, outputs voltage Ua (in V)
    PID_Update(&State_MotorOmega_L, &Para_MotorOmega_L, Speed_FB.Left_Speed);
    PID_Update(&State_MotorOmega_R, &Para_MotorOmega_R, Speed_FB.Right_Speed);

    float ua_l = PID_GetOutput(&State_MotorOmega_L);
    float ua_r = PID_GetOutput(&State_MotorOmega_R);

    // Read battery voltage, to compensate for voltage changes
    float vbat = Battery_GetVoltage();
    if(vbat < 6.0f) vbat = 6.0f;  // Prevent dividing by a near-zero value

    // Debug print: check the actual battery voltage
    // char buf[32];
    // sprintf(buf, "vbat:%.2f\r\n", vbat);
    // Debugging_USART_SendString(buf);

    // Convert to a PWM duty-cycle output
    int16_t duty_l = (int16_t)(ua_l / vbat * 100.0f);
    int16_t duty_r = (int16_t)(ua_r / vbat * 100.0f);

    // Debug print: check target/feedback/PID output/final duty cycle
    // char buf[80];
    // sprintf(buf, "SP_L:%.2f FB_L:%.2f ua_l:%.2f duty_l:%d\r\n",
    //         State_MotorOmega_L.SP, Speed_FB.Left_Speed, ua_l, duty_l);
    // Debugging_USART_SendString(buf);

    Motor_Speed_Set(duty_l, duty_r);
}


/**
 * [Algorithm layer] Set the target speed (external Set interface, called by Bluetooth/external sources)
 * Receives an external speed command, writes it into the velocity loop's SP
 */
void Control_SetMoveSpeed(float speed)
{
    PID_SetSP(&State_Velocity, speed);
}

/**
 * [Algorithm layer] Reset all PID state (external Reset interface)
 * Zeroes out the runtime state of the three loops, doesn't affect already-tuned parameters
 * When it's called: mode switching, error recovery, restarting balance
 */
void Control_Reset(void)
{
    omega_ref = 0.0f;
    PID_Reset(&State_Velocity);
    PID_Reset(&State_Theta);
    PID_Reset(&State_ThetaDot);
    PID_Reset(&State_MotorOmega_L);
    PID_Reset(&State_MotorOmega_R);
    PID_Reset(&State_Turn);

}

