#ifndef	__ENCODERFEEDBACK_H
#define	__ENCODERFEEDBACK_H

#include "stm32f10x.h"
#include "Motor_Encoder.h"

/**
 * ================================================================
 * [Five-Layer Tower Placement] Feedback processing layer - Encoder speed feedback module
 * ================================================================
 *
 * This module's position in the five-layer tower:
 *
 *   Algorithm layer → Calls EncoderFeedback_GetSpeed() to get wheel speed
 *   Feedback layer  ← This module, converts encoder pulse increments into wheel speed in physical units
 *   Driver layer    → This module calls Motor_Encoder's Encoder_Get_L/R_Speed()
 *
 *   The driver layer's Encoder_Get_L_Speed() returns "the pulse increment
 *   within each 5ms" (a plain number, no physical unit)
 *   The algorithm layer's PID needs "wheel speed (rad/s)" or "linear speed
 *   (m/s)" (physically meaningful)
 *   The feedback layer is responsible for this "raw number → physical
 *   quantity" translation, following the same layering principle as
 *   ComplementaryFilter converting "raw LSB → angle"
 *
 *   Encoder line count = 22 (single edge of phase A, 22 pulses per revolution)
 *   Reduction ratio     = 30613/1500 ≈ 20.4
 *   Total pulses per wheel revolution = 22 × 20.4 ≈ 449
 */

#define ENCODER_LINE_COUNTE                         22
#define REDUCTION_RATIO                             (30613.0f / 1500.0f)
#define TOTAL_NUM_OF_PULSES_PER_WHEEL_REVOLUTION    449
#define FEEDBACK_DT                                 0.005f  // Call period, in seconds


typedef struct
{
    /* data */
    float Left_Speed;
    float Right_Speed;
} SpeedFeedback;


void EncoderFeedback_Update(void);   // Called every 5ms, reads the pulse increment, converts it into wheel speed

void EncoderFeedback_GetSpeed(SpeedFeedback* Outputs); // Returns the average wheel speed (m/s)



#endif

