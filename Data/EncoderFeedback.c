#include "EncoderFeedback.h"

/* ============================================================
 * VAR_INPUT (read internally via Encoder_Get_L/R_Speed()):
 *   Pulse increment (int32_t) → encoder count change within each 5ms
 *
 * VAR_OUTPUT (exposed externally via GetSpeed()):
 *   Speed_Output.Left_Speed  → left wheel speed, in rad/s
 *   Speed_Output.Right_Speed → right wheel speed, in rad/s
 *
 * VAR (internal state):
 *   Speed_Output → cache of the latest converted result
 * ============================================================ */

static SpeedFeedback Speed_Output;   // VAR: internal cache, exposed externally via GetSpeed()

/**
 * [Feedback layer] Encoder speed update (external Update interface, called by Task_Manager every 5ms)
 *
 * Conversion formula:
 *   Wheel speed (rad/s) = pulse increment / (line count × reduction ratio) × 2π / dt
 *   Physical meaning: pulse increment ÷ total pulses = revolutions turned, × 2π = radians, ÷ dt = rad/s
 *
 */
void EncoderFeedback_Update(void)
{
    Speed_Output.Left_Speed  = (float)Encoder_Get_L_Speed();
    Speed_Output.Right_Speed = (float)Encoder_Get_R_Speed();
}

/**
 * [Feedback layer] Get wheel speed (external Get interface)
 *
 * Responsibility: copies the internal Speed_Output struct to the caller, doesn't trigger any computation
 *
 *   The return form suits a single value
 *   The pointer form suits a struct (multiple values packed together)
 *   Fundamentally the same: both are a read-only window onto internal state
 *
 */
void EncoderFeedback_GetSpeed(SpeedFeedback* Outputs)
{
    *Outputs = Speed_Output;
}



