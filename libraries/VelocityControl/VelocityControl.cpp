/*
    VelocityControl.cpp

    Responsibility:
    ---------------
    Control the vehicle's velocity based on a requested cruise velocity.
    The driver is assumed to use analog outputs controlling the driving wheels.

    An instances of this class is an "active object" and
    it's "Update" function must be called on a regular basis.
*/

#include "VelocityControl.h"
#include "Debug.h"
#include "HwWrap.h"

VelocityControl::VelocityControl
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    output.VelocityStop();
}

void VelocityControl::Set
//  --------------------------------------------------------------------------------
(
    float velocityRequest, // The requested velocity fraction of the vehicle
    float velocityRamp     // The requested increase or decrease value per update
)
//  --------------------------------------------------------------------------------
{
    velocityRequested = velocityRequest;
    velocityRampRequested = velocityRamp;
}

float VelocityControl::Update
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    if (velocityCurrentValue < velocityRequested)
    {
        velocityCurrentValue += velocityRampRequested;

        if (velocityCurrentValue > velocityRequested)
        {
            velocityCurrentValue = velocityRequested;
        }
    }
    else if (velocityCurrentValue > velocityRequested)
    {
        velocityCurrentValue -= velocityRampRequested;

        if (velocityCurrentValue < velocityRequested)
        {
            velocityCurrentValue = velocityRequested;
        }
    }
    return velocityCurrentValue;
}

void VelocityControl::DebugInfo
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    HwWrap::GetInstance()->DebugString(" Velocity Control: ");
    if (velocityRequested >= 0.0)
        HwWrap::GetInstance()->DebugString(" ");
    HwWrap::GetInstance()->DebugFloat(velocityRequested);

    HwWrap::GetInstance()->DebugString(" -> ");
    if (velocityCurrentValue >= 0.0)
        HwWrap::GetInstance()->DebugString(" ");
    HwWrap::GetInstance()->DebugFloat(velocityCurrentValue);

#ifndef VELOCITY_USE_DEBUGGING_PID
    HwWrap::GetInstance()->DebugNewLine();
#endif
}
