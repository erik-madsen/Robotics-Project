/* 
    PIDregulator.cpp
    The file contains the definition of a class used for PID regulation.
*/

/*
Functional description :
------------------------
Until the PID regulator is enabled, the output is 0.0
The default range of the output from the PID regulator 
is [0.0 .. 1.0] but it can be changed to [-1.0 .. 1.0]

Trimming the PID regulator :
----------------------------
A simple method to achieve an initial, usable setting of 
the PID regulator parameters is as follows:
 - Set Kp, Ki, and Kd to zero.
 - Gradually increase the Kp until the output of the PID regulator
   becomes unstable.
   Then use a value of Kp that is approximately 50% of this.
 - Gradually increase the Ki until the output of the PID regulator
   becomes unstable.
   Then use a value of Ki that is approximately 80% of this.
 - Gradually increase the Kd until the output of the PID regulator
   becomes unstable.
   Then use a value of Kd that is approximately 80% of this.
The PID regulator may afterwards need a mores specific adjustment 
with respect to the actual regulation loop and it purpose.

Special notes :
---------------
The limit on the summarized error is set so that the iTerm
will not exceed the maximum output of the regulator
*/

#include "PIDregulator.h"
#include "Debug.h"
#include "HwWrap.h"

PIDregulator::PIDregulator(void)
{
    enabled = false;
    rangeToMinusOne = false;

    Kp = 0.10f;
    Ki = 0.05f;
    Kd = 0.00f;

    errorSum = 0.0f;
    errorSumMax = 1.0f / Ki;
    errorLast = 0.0f;
}

void PIDregulator::ResetInternalValues(void)
{
    errorSum = 0.0f;
    errorSumMax = 1.0f / Ki;
    errorLast = 0.0f;
}

void PIDregulator::Init(void)
{
    enabled = true;
}

void PIDregulator::SetRangeToIncludeMinusOne(bool setting)
{
    rangeToMinusOne = setting;
}

void PIDregulator::SetKp(float value)
{
    Kp = value;
}

void PIDregulator::SetKi(float value)
{
    Ki = value;

    if (Ki != 0.0)
    {
        errorSumMax = 1.0 / Ki;
    }
}

void PIDregulator::SetKd(float value)
{
    Kd = value;
}

void PIDregulator::GetParams(t_PID_parameters* param_ptr)
{
    // OS_EnterRegion();
    // ... just to make sure that the parameters are coherent

    param_ptr->Kp = Kp;
    param_ptr->Ki = Ki;
    param_ptr->Kd = Kd;

    param_ptr->pTerm = pTerm;
    param_ptr->iTerm = iTerm;
    param_ptr->dTerm = dTerm;

    param_ptr->errorSum = errorSum;

    // OS_LeaveRegion();
}

float PIDregulator::Update
(
    float error  // the actual error of the regulated parameter in the range [-1.0 .. 1.0]
)
{
    if (!enabled)
    {
        error = 0.0;
    }

    if (error >  1.0) error =  1.0;
    if (error < -1.0) error = -1.0;

    errorSum += error;

    if (errorSum >  errorSumMax) errorSum = errorSumMax;
    if (rangeToMinusOne)
    {
        if (errorSum < -errorSumMax) errorSum = -errorSumMax;
    } 
    else
    {
        if (errorSum <  0.0)  errorSum =  0.0;
    }

    pTerm = Kp * error;
    iTerm = Ki * errorSum;
    dTerm = Kd * (error - errorLast);

    errorLast = error;

    output = pTerm + iTerm + dTerm;

    if (output >  1.0) output = 1.0;
    if (rangeToMinusOne)
    {
        if (output < -1.0) output = -1.0;
    }
    else
    {
        if (output <  0.0) output = 0.0;
    }

    return output;
}

void PIDregulator::DebugInfo(void)
{
    HwWrap::GetInstance()->DebugString(" PID:  ");
    HwWrap::GetInstance()->DebugString("error ");
    HwWrap::GetInstance()->DebugFloat(errorLast);
    HwWrap::GetInstance()->DebugString("  output ");
    HwWrap::GetInstance()->DebugFloat(output);
    HwWrap::GetInstance()->DebugNewLine();
}
