#include "stm32f10x.h"
#include "PID.h"

/**
 * [Algorithm layer] PID initialization (external Action interface)
 *
 *   Init's responsibility is "writing parameters into the struct from the
 *   outside" — loose parameters are passed in, Init packs and stores them,
 *   the caller only needs to know the values
 *   Analogous to CODESYS: filling in an FB's parameters field by field,
 *   not passing the whole VAR block in
 *
 *   But the three loops may be called at different frequencies (the speed
 *   loop at 50ms, the angle/angular-velocity loops at 5ms)
 *   A single fixed macro can't serve loops running at different frequencies
 *   Putting DT inside Para_t lets each loop pass in its own call period at
 *   initialization
 *   Only then can the same PID code actually be reused across loops of
 *   different frequencies
 */
void PID_Init(PID_Para_t* para, float kp, float ki, float kd,
              float upper, float lower, float dt)
{
    para->KP = kp;
    para->KI = ki;
    para->KD = kd;
    para->Integral_UpperLimit = upper;
    para->Integral_LowerLimit = lower;
    para->DT = dt;
}

/**
 * [Algorithm layer] Set the target value (external Set interface)
 *
 * Responsibility: called by an outer loop, writes its own output into the inner loop's SP
 *
 */
void PID_SetSP(PID_State_t* state, float sp)
{
    state->SP = sp;
}

/**
 * [Algorithm layer] PID computation update (external Update interface, called every control period)
 *
 * Discrete PID formula:
 *   err      = SP - FB                        (present: proportional basis)
 *   err_int += err × dt                       (past: integral accumulation)
 *   err_dev  = (err - err_prev) / dt          (future: derivative prediction)
 *   Output   = Kp×err + Ki×err_int + Kd×err_dev
 *
 * [Design decision regarding output clamping]
 *   Output clamping was originally placed inside PID_Update()
 *   Later realized: the output range is a system constraint (the motor's
 *   physical limits), not part of the PID algorithm itself
 *   PID is a pure computational tool and shouldn't know about the
 *   actuator's physical limits
 *   Final decision: output clamping is Control.c's responsibility, after
 *   it calls PID_GetOutput()
 *   Integral clamping stays inside Update() — that's anti-windup, which is
 *   internal to the algorithm
 */
void PID_Update(PID_State_t* state, PID_Para_t* para, float FB)
{
    /* 1. Compute the current error */
    state->Error = state->SP - FB;

    /* First-call guard: only record the error, don't compute an output */
    if(state->is_first)
    {
        state->is_first = 0;
        state->Error_Previous = state->Error;  // Record the current error
        return;
    }

    float Error_Derivative;

    /* 2. Integral accumulation (trapezoidal integration) */
    state->Error_Integral += (state->Error + state->Error_Previous) * para->DT * 0.5f;

    /* 3. Derivative computation */
    Error_Derivative = (state->Error - state->Error_Previous) / para->DT;

    /* 4. Compute the output */
    state->Output = para->KP * state->Error
                  + para->KI * state->Error_Integral
                  + para->KD * Error_Derivative;

    /* 5. Integral clamping */
    if(state->Error_Integral > para->Integral_UpperLimit)
        state->Error_Integral = para->Integral_UpperLimit;
    if(state->Error_Integral < para->Integral_LowerLimit)
        state->Error_Integral = para->Integral_LowerLimit;

    /* 6. Update the previous error */
    state->Error_Previous = state->Error;
}

/**
 * [Algorithm layer] Read the PID output (external Get interface); the "export" operation for VAR_OUTPUT
 */
float PID_GetOutput(PID_State_t* state)
{
    return state->Output;
}

/**
 * [Algorithm layer] Reset PID state (external Reset interface)
 *
 * Zeroes out all runtime state, doesn't affect the already-configured parameters
 * Analogous to CODESYS: resetting an FB only clears VAR, doesn't touch VAR_CONSTANT
 */
void PID_Reset(PID_State_t* state)
{
    state->SP             = 0;
    state->Error          = 0;
    state->Error_Integral = 0;
    state->Error_Previous = 0;
    state->Output         = 0;
    state->is_first       = 1;
}


