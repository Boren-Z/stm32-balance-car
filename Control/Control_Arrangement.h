#ifndef	__CONTROL_ARRANGEMENT_H
#define	__CONTROL_ARRANGEMENT_H

#include "stm32f10x.h"


/**
 * [Five-Layer Tower Placement] Algorithm layer - Cascaded PID control logic module
 *
 * This module's position in the five-layer tower:
 *
 *   Scheduling layer → Calls Control_Update() (every 5ms), Control_Motor_Update() (every 1ms), and Control_SetMoveSpeed()
 *   Algorithm layer  ← This module, cascades four PID loops, ultimately driving the motors
 *   Feedback layer   → This module calls ComplementaryFilter's and EncoderFeedback's Get interfaces
 *   Driver layer     → This module calls Motor_Speed_Set() to output the control quantity
 *
 *   Doesn't do the actual math (that's PID.c's job)
 *   Doesn't read sensors (that's the feedback layer's job)
 *   Only responsible for: passing the right data, at the right time, to the right loop
 *   Analogous to a CODESYS main program: calling each FB instance, chaining their inputs and outputs together
 *
 * [Four-loop cascade structure and physical meaning]
 *   Velocity loop:
 *     FB = actual linear speed (encoder)
 *     SP = target speed (set via Bluetooth, =0 at rest)
 *     Output = target tilt angle (theta_ref)
 *     Physical meaning: the car needs to lean back to decelerate, and lean forward to accelerate
 *
 *   Angle loop:
 *     FB = actual tilt angle (complementary filter)
 *     SP = target tilt angle output by the velocity loop
 *     Output = target angular velocity (theta_dot_ref)
 *
 *   Angular velocity loop:
 *     FB = actual angular velocity (gyroscope GyroX)
 *     SP = target angular velocity output by the angle loop
 *     Output = target angular acceleration, ultimately converted into a target wheel speed (omega_ref)
 *
 *   Motor speed loop (4th loop, runs at 1ms in Control_Motor_Update(), faster than the other three which run at 5ms):
 *     FB = actual wheel speed (encoder)
 *     SP = omega_ref, the target wheel speed output by the three-loop cascade above (± a turn-loop correction)
 *     Output = voltage Ua, converted to a PWM duty cycle by dividing by battery voltage
 *     Why it's needed: the three-loop cascade only produces a target wheel
 *     speed, not a PWM value — going from wheel speed to PWM open-loop
 *     would drift as the battery voltage drops; this loop closes that gap
 *
 * [Why the loops don't "fight" each other]
 *   The key to cascaded PID: an outer loop's output is the inner loop's SP, it never controls the motor directly
 *   Only the innermost loop's output ever actually controls the motor, so there's no scenario where two loops compete for the PWM value
 *   An inner loop must be faster than its outer loop (responding to falling over needs to be faster than responding to drift)
 *   This is dictated by the physical time scales involved, not an arbitrary rule
 *
 */
void Control_Init(void);

void Control_Update(void);

void Control_SetMoveSpeed(float speed);

void Control_Motor_Update(void);

void Control_Reset(void);

#endif


