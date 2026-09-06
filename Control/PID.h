#ifndef	__PID_H
#define	__PID_H
#include "stm32f10x.h"

/**
 * [Five-Layer Placement] Algorithm layer - Generic PID controller module
 *
 * This module's position in the five layers:
 *
 *   Scheduling layer → Doesn't call PID directly, drives it indirectly via the Control layer
 *   Algorithm layer  ← This module, provides a reusable discrete PID computation unit
 *   Feedback layer   → The Control layer reads the FB value from the feedback layer, then passes it into PID_Update()
 *
 * Core design
 *   PID is a pure computational tool — it doesn't know which loop it's
 *   used in
 *   Doesn't know the motor's physical limits, doesn't know the vehicle
 *   body's physical parameters
 *   Does exactly one thing: given SP and FB, compute Output
 *   This is the "single responsibility principle" applied to a control algorithm
 *   The balance car needs three PID loops (velocity/angle/angular velocity)
 *   Same algorithm logic, three independent sets of parameters and state
 *   Must pass a struct pointer to distinguish "which instance is being operated on"
 *   Analogous to CODESYS: the same FB type, instantiated three times
 *     pid_velocity : PID_FB;
 *     pid_theta    : PID_FB;
 *     pid_theta_dot: PID_FB;
 *
 * External interface:
 *   PID_Init()      → Action interface, initializes parameters
 *   PID_SetSP()     → Set interface, an outer loop writes the target value
 *   PID_Update()    → Update interface, advances the computation by one step
 *   PID_GetOutput() → Get interface, reads the latest output
 *   PID_Reset()     → Action interface, resets internal state
 *
 * Division of labor between the structs
 *   PID_Para_t  → VAR_CONSTANT: Kp/Ki/Kd/DT/integral clamping limits
 *                 Unchanged after initialization; switching parameters
 *                 means switching to a different tuning profile
 *   PID_State_t → VAR: SP/error terms/integral/Output
 *                 Runtime state that changes every period
 *   Reason: resetting only clears State, not Para — parameters that have
 *   already been tuned shouldn't be lost just because the system resets
 */

typedef struct
{
    float KP;
    float KI;
    float KD;
    float DT;
    float Integral_UpperLimit;  // Integral clamping limit (anti-windup, internal to the algorithm)
    float Integral_LowerLimit;

} PID_Para_t;

typedef struct
{
    /* data */
    // VAR: runtime state, updated every period
    float SP;             // VAR_INPUT: written by an outer loop via PID_SetSP()

    float Error_Previous;
    float Error_Integral;
    float Error;

    float Output;

    uint8_t is_first;  // First-call flag
} PID_State_t;


void  PID_Init(PID_Para_t* para, float kp, float ki, float kd, float upper, float lower, float dt);

void  PID_SetSP(PID_State_t* state, float sp);

void  PID_Update(PID_State_t* state, PID_Para_t* para, float FB);

float PID_GetOutput(PID_State_t* state);

void  PID_Reset(PID_State_t* state);


#endif

