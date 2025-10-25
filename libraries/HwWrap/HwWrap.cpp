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


void HwWrap_VelocityOutput::VelocityStop
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
#ifdef VELOCITY_USE_OUTPUTS
    analogWrite(velocityInADriveBackwards, 0);
    analogWrite(velocityInBDriveForwards, 0);
#endif
}

void HwWrap_VelocityOutput::VelocityFwd
//  --------------------------------------------------------------------------------
(
    float fraction
)
//  --------------------------------------------------------------------------------
{
#ifdef VELOCITY_USE_OUTPUTS
    analogWrite(velocityInADriveBackwards, 0);
    analogWrite(velocityInBDriveForwards, (unsigned char)(float(PWM_RANGE-1) * fraction));
#endif
}

void HwWrap_VelocityOutput::VelocityBwd
//  --------------------------------------------------------------------------------
(
    float fraction
)
//  --------------------------------------------------------------------------------
{
#ifdef VELOCITY_USE_OUTPUTS
    analogWrite(velocityInADriveBackwards, (unsigned char)(float(PWM_RANGE-1) * fraction));
    analogWrite(velocityInBDriveForwards, 0);
#endif
}


void HwWrap_SteeringOutput::SteeringStraight
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

void HwWrap_SteeringOutput::SteeringRight
//  --------------------------------------------------------------------------------
(
    float fraction
)
//  --------------------------------------------------------------------------------
{
#ifdef STEERING_USE_OUTPUTS
    analogWrite(steeringInATurnRight, (unsigned char)(float(PWM_RANGE-1) * fraction));
    analogWrite(steeringInBTurnLeft, 0);
#endif
}

void HwWrap_SteeringOutput::SteeringLeft
//  --------------------------------------------------------------------------------
(
    float fraction
)
//  --------------------------------------------------------------------------------
{
#ifdef STEERING_USE_OUTPUTS
    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, (unsigned char)(float(PWM_RANGE-1) * fraction));
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

void HwWrap::DebugInt
//  --------------------------------------------------------------------------------
(
    int value
)
//  --------------------------------------------------------------------------------
{
    Serial.print(value);
}

void HwWrap::DebugLong
//  --------------------------------------------------------------------------------
(
    long value
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
