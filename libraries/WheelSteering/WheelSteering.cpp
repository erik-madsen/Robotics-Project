/*
    WheelSteering.cpp

    Responsibility:
    Control the steering wheels of the veichle based on a requested steering amount.
    The driver is assumed to use DOs controlling a pair of rather slowly reacting steering wheels.
    Hence the control is performing a "programmed PWM" with 10 differnt duty cycles.
*/

#include "WheelSteering.h"
#include "Debug.h"
#include "HwWrap.h"

#define NO_OF_DUTY_CYCLES 10.0f

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
    // Control the "PWM periods

    if (timeSlot++ >= unsigned(NO_OF_DUTY_CYCLES))
    {
        timeSlot = 1;
        steeringSignalInUse = steeringSignalRequested;

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

    // Control the "PWM duty cycle

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

void WheelSteering::DebugInfo
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    if (timeSlot == 1)
        HwWrap::GetInstance()->DebugString(" Steering: ");
    else
        HwWrap::GetInstance()->DebugString("           ");

    if (steeringSignalInUse >= 0.0f)
        HwWrap::GetInstance()->DebugString(" ");
    HwWrap::GetInstance()->DebugFloat(steeringSignalInUse);
    HwWrap::GetInstance()->DebugString("                   ");

    if (currentWheelPos == steeringDirection_STRAIGHT)
        HwWrap::GetInstance()->DebugString("|| ***** ||");
    else if (currentWheelPos == steeringDirection_LEFT)
        HwWrap::GetInstance()->DebugString("\\\\ ***** \\\\");
    else if (currentWheelPos == steeringDirection_RIGHT)
        HwWrap::GetInstance()->DebugString("// ***** //");

    if (timeSlot < 10)
        HwWrap::GetInstance()->DebugString(" ");
    HwWrap::GetInstance()->DebugString("  ");
    HwWrap::GetInstance()->DebugUnsigned(timeSlot);
    HwWrap::GetInstance()->DebugString("  ");
    HwWrap::GetInstance()->DebugFloat(float(timeSlot) / NO_OF_DUTY_CYCLES);

    HwWrap::GetInstance()->DebugNewLine();
}
