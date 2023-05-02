// A HW wrapper class for I/O etc.
// The class uses the singleton pattern.

#include "HwWrap.h"
#include "Arduino.h"

HwWrap::HwWrap(void)
{
  my_instance = this;
}

void HwWrap::Init(void)
{
}


unsigned HwWrap::AnalogInput(int inputNo)
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


unsigned HwWrap::DigitalInput(int inputNo)
{
    return digitalRead(inputNo);
}


void HwWrap::MotionStop()
{
  digitalWrite(motionInADriveBackwards, LOW);
  digitalWrite(motionInBDriveForwards, LOW);
}

void HwWrap::MotionFwd()
{
  digitalWrite(motionInADriveBackwards, LOW);
  digitalWrite(motionInBDriveForwards, HIGH);
}

void HwWrap::MotionBwd()
{
  digitalWrite(motionInADriveBackwards, HIGH);
  digitalWrite(motionInBDriveForwards, LOW);
}


void HwWrap::DebugString(const char *string)
{
  Serial.print(string);
}

void HwWrap::DebugUnsigned(unsigned value)
{
  Serial.print(value);
}

void HwWrap::DebugFloat(float value)
{
  Serial.print(value);
}

void HwWrap::DebugNewLine()
{
  Serial.println();
}
