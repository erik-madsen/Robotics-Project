/*
    WheelSteering.cpp

    Responsibility:
    ---------------
    Control the steering wheels of the veichle based on a requested steering amount.
    The driver is assumed to use DOs controlling a pair of rather slowly reacting steering wheels.
    Hence the control is performing a "programmed PWM" with NO_OF_DUTY_CYCLES possible duty cycles.

    An instances of this class is an "active object" and
    it's "Update" function must be called on a regular basis.
*/

#include "WheelSteering.h"
#include "Debug.h"
#include "HwWrap.h"
#include "math.h"

#define NO_OF_DUTY_CYCLES 5.0f

WheelSteering::WheelSteering
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
}

void WheelSteering::Set
//  --------------------------------------------------------------------------------
(
    float steeringRequest  // The requested steering amount of the veichle
)
//  --------------------------------------------------------------------------------
{
    steeringSignalRequested = steeringRequest;

    if (currentDirection == steeringDirection_STRAIGHT)
    {
        // Start the next steering periode immediately
        timeSlot = unsigned(NO_OF_DUTY_CYCLES);
    }
}

void WheelSteering::Update
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    // Control the "PWM periods"

    if (timeSlot++ >= unsigned(NO_OF_DUTY_CYCLES))
    {
        timeSlot = 1;

        steeringSignalInUse = float( round(steeringSignalRequested * NO_OF_DUTY_CYCLES) ) / NO_OF_DUTY_CYCLES;

        if (steeringSignalInUse >= (1.0f / NO_OF_DUTY_CYCLES))
        {
            currentDirection = steeringDirection_RIGHT;
            currentWheelPos = steeringDirection_RIGHT;
            HwWrap::GetInstance()->SteeringRight();
        }
        else if (steeringSignalInUse <= -(1.0f / NO_OF_DUTY_CYCLES))
        {
            currentDirection = steeringDirection_LEFT;
            currentWheelPos = steeringDirection_LEFT;
            HwWrap::GetInstance()->SteeringLeft();
        }
        else
        {
            currentDirection = steeringDirection_STRAIGHT;
            currentWheelPos = steeringDirection_STRAIGHT;
            HwWrap::GetInstance()->SteeringStraight();
        }
    }

    // Control the "PWM duty cycle"

    switch (currentDirection)
    {
        case steeringDirection_RIGHT:
        {
            if ((float(timeSlot) / NO_OF_DUTY_CYCLES) <= steeringSignalInUse)
            {
                HwWrap::GetInstance()->SteeringRight();
                currentWheelPos = steeringDirection_RIGHT;
            }
            else
            {
                HwWrap::GetInstance()->SteeringStraight();
                currentWheelPos = steeringDirection_STRAIGHT;
            }
        }
        break;

        case steeringDirection_LEFT:
        {
            if ((float(timeSlot) / NO_OF_DUTY_CYCLES) <= -steeringSignalInUse)
            {
                HwWrap::GetInstance()->SteeringLeft();
                currentWheelPos = steeringDirection_LEFT;
            }
            else
            {
                HwWrap::GetInstance()->SteeringStraight();
                currentWheelPos = steeringDirection_STRAIGHT;
            }
        }
        break;

        case steeringDirection_STRAIGHT:
        default:
        {
            HwWrap::GetInstance()->SteeringStraight();
            currentWheelPos = steeringDirection_STRAIGHT;
        }
        break;
    }
}

void WheelSteering::DebugInfoGraphics
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    HwWrap::GetInstance()->DebugString("  ");
    if (timeSlot < 10)
        HwWrap::GetInstance()->DebugString(" ");
    HwWrap::GetInstance()->DebugUnsigned(timeSlot);
    HwWrap::GetInstance()->DebugString("  ");
    HwWrap::GetInstance()->DebugFloat(float(timeSlot) / NO_OF_DUTY_CYCLES);

    HwWrap::GetInstance()->DebugString("   ");
    if (currentWheelPos == steeringDirection_STRAIGHT)
        HwWrap::GetInstance()->DebugString("|| ----- ||");
    else if (currentWheelPos == steeringDirection_LEFT)
        HwWrap::GetInstance()->DebugString("\\\\ ----- \\\\");
    else if (currentWheelPos == steeringDirection_RIGHT)
        HwWrap::GetInstance()->DebugString("// ----- //");
}

void WheelSteering::DebugInfo
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    if (timeSlot == 1)
    {
        HwWrap::GetInstance()->DebugString(" Steering: ");
        if (steeringSignalInUse >= 0.0f)
            HwWrap::GetInstance()->DebugString(" ");
        HwWrap::GetInstance()->DebugFloat(steeringSignalRequested);

        HwWrap::GetInstance()->DebugString(" -> ");
        if (steeringSignalInUse >= 0.0f)
            HwWrap::GetInstance()->DebugString(" ");
        HwWrap::GetInstance()->DebugFloat(steeringSignalInUse);

#ifdef STEERING_USE_DEBUGGING_VERBOSE
        DebugInfoGraphics();
#endif
        HwWrap::GetInstance()->DebugNewLine();
    }
    else
    {
#ifdef STEERING_USE_DEBUGGING_VERBOSE
        HwWrap::GetInstance()->DebugString("                         ");
        DebugInfoGraphics();
        HwWrap::GetInstance()->DebugNewLine();
#endif
    }

}
