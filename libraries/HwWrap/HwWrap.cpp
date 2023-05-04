/*
    HwWrap.cpp

    Responsibility:
    ---------------
    Implement a HW wrapper class for I/O etc.
    The class uses the singleton pattern.
*/

#include "Debug.h"
#include "stdint.h"
#include "HwWrap.h"
#include "Arduino.h"

HwWrap::HwWrap
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    my_instance = this;
}

void HwWrap::Init
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
}


unsigned HwWrap::AnalogInput
//  --------------------------------------------------------------------------------
(
    uint8_t inputNo  // Arduino port number
)
//  --------------------------------------------------------------------------------
{
    unsigned value;

    switch (inputNo)
    {
        case 0:
            value = analogRead(A0);
            break;
        case 1:
            value = analogRead(A1);
            break;
        case 2:
            value = analogRead(A2);
            break;
        case 3:
            value = analogRead(A3);
            break;
        case 4:
            value = analogRead(A4);
            break;
    }

    return value;
}

void HwWrap::AnalogOutput
//  --------------------------------------------------------------------------------
(
    uint8_t outputNo,   // Arduino port number
    unsigned int value  // PWM dutycycle in the range [0..255]
)
//  --------------------------------------------------------------------------------
{
    analogWrite(outputNo, value);
}

unsigned HwWrap::DigitalInput
//  --------------------------------------------------------------------------------
(
    uint8_t inputNo  // Arduino port number
)
//  --------------------------------------------------------------------------------
{
    return digitalRead(inputNo);
}


void HwWrap::MotionStop
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef MOTION_CONTROL_USE_OUTPUTS
    digitalWrite(motionInADriveBackwards, LOW);
    digitalWrite(motionInBDriveForwards, LOW);
#endif
}

void HwWrap::MotionFwd
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef MOTION_CONTROL_USE_OUTPUTS
    digitalWrite(motionInADriveBackwards, LOW);
    digitalWrite(motionInBDriveForwards, HIGH);
#endif
}

void HwWrap::MotionBwd
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef MOTION_CONTROL_USE_OUTPUTS
    digitalWrite(motionInADriveBackwards, HIGH);
    digitalWrite(motionInBDriveForwards, LOW);
#endif
}


void HwWrap::SteeringStraight
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef STEERING_USE_OUTPUTS
    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, 0);
#endif
}

void HwWrap::SteeringRight
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef STEERING_USE_OUTPUTS
    analogWrite(steeringInATurnRight, (PWM_RANGE-1));
    analogWrite(steeringInBTurnLeft, 0);
#endif
}

void HwWrap::SteeringLeft
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef STEERING_USE_OUTPUTS
    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, (PWM_RANGE-1));
#endif
}


void HwWrap::DebugString
//  --------------------------------------------------------------------------------
(
    const char *string
)
//  --------------------------------------------------------------------------------
{
    Serial.print(string);
}

void HwWrap::DebugUnsigned
//  --------------------------------------------------------------------------------
(
    unsigned value
)
//  --------------------------------------------------------------------------------
{
    Serial.print(value);
}

void HwWrap::DebugFloat
//  --------------------------------------------------------------------------------
(
    float value
)
//  --------------------------------------------------------------------------------
{
    Serial.print(value);
}

void HwWrap::DebugNewLine
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    Serial.println();
}
