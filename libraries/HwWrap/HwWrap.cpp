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
    digitalWrite(motionInADriveBackwards, LOW);
    digitalWrite(motionInBDriveForwards, LOW);
}

void HwWrap::MotionFwd
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    digitalWrite(motionInADriveBackwards, LOW);
    digitalWrite(motionInBDriveForwards, HIGH);
}

void HwWrap::MotionBwd
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    digitalWrite(motionInADriveBackwards, HIGH);
    digitalWrite(motionInBDriveForwards, LOW);
}


void HwWrap::SteeringStraight
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, 0);
}

void HwWrap::SteeringRight
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    analogWrite(steeringInATurnRight, (PWM_RANGE-1));
    analogWrite(steeringInBTurnLeft, 0);
}

void HwWrap::SteeringLeft
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, (PWM_RANGE-1));
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
