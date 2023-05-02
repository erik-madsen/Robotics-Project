/* 
    WheelSteering.h

    Responsibility:
    Control the steering wheels of the veichle.
*/

#include "WheelSteering.h"
#include "Debug.h"
#include "HwWrap.h"

WheelSteering::WheelSteering(void)
{
}

void WheelSteering::Update
(
    float steeringSignal  // The requested steering amount of the veichle
)
{
    if ((steeringSignal) > 0.5)
    {
        currentDir = steeringDirection_RIGHT;
        HwWrap::GetInstance()->AnalogOutput(steeringInATurnRight, steeringSignal * PWM_GAIN * (PWM_RANGE-1));
        HwWrap::GetInstance()->AnalogOutput(steeringInBTurnLeft, 0);
    }
    else if ((steeringSignal) < -0.5)
    {
        currentDir = steeringDirection_LEFT;
        HwWrap::GetInstance()->AnalogOutput(steeringInATurnRight, 0);
        HwWrap::GetInstance()->AnalogOutput(steeringInBTurnLeft, (-steeringSignal) * PWM_GAIN * (PWM_RANGE-1));
    }
    else
    {
        currentDir = steeringDirection_STRAIGHT;
        HwWrap::GetInstance()->AnalogOutput(steeringInATurnRight, 0);
        HwWrap::GetInstance()->AnalogOutput(steeringInBTurnLeft, 0);
    }
}

void WheelSteering::DebugInfo(void)
{
    HwWrap::GetInstance()->DebugString(" Steering:                                 ");
    if (currentDir == steeringDirection_STRAIGHT)
        HwWrap::GetInstance()->DebugString("| |");
    else if (currentDir == steeringDirection_LEFT)
        HwWrap::GetInstance()->DebugString("\\ \\");
    else if (currentDir == steeringDirection_RIGHT)
        HwWrap::GetInstance()->DebugString("/ /");
    HwWrap::GetInstance()->DebugNewLine();
}
