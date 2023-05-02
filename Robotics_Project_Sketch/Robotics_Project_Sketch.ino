/*
  An Arduino IDE SW for experimenting with the line tracking module controlling the steering
*/

#include "Common.h"
#include "HwWrap.h"
#include "LineTracker.h"
#include "PIDregulator.h"

float PIDoutput;

int debugPrescaler = 0;

/* Pin description */

int steeringInA = 2;
int steeringInB = 3;
int motionInA   = 40;
int motionInB   = 41;


void setup()
{
  Serial.begin(9600);
  while (!Serial);

  LineTracker_Init();

  PID_constructor();
  PID_SetRangeToIncludeMinusOne(1);
  PID_SetKp(1.8);
  PID_SetKi(0.0);
  PID_Init();
}


unsigned HwWrap_AnalogInput(unsigned inputNo)
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
}

void HwWrap_MotionStop()
{
  digitalWrite(motionInA, LOW);
  digitalWrite(motionInB, LOW);
}

void HwWrap_MotionFwd()
{
  digitalWrite(motionInA, LOW);
  digitalWrite(motionInB, HIGH);
}

void HwWrap_MotionBwd()
{
  digitalWrite(motionInA, HIGH);
  digitalWrite(motionInB, LOW);
}


void HwWrap_DebugString(char *string)
{
  Serial.print(string);
}

void HwWrap_DebugUnsigned(unsigned value)
{
  Serial.print(value);
}

void HwWrap_DebugFloat(float value)
{
  Serial.print(value);
}

void HwWrap_DebugNewLine()
{
  Serial.println();
}


void loop()
{
  t_boolean lineDetected;
  float position;

  LineTracker_Update(&lineDetected, &position);

  if (lineDetected)
  {
    /* Control steering using PID regulator */
  
    PIDoutput = PID_Update(0.0 - position);
  
    if (PIDoutput > 0.0)
    {
      analogWrite(steeringInA, PIDoutput * (PWM_RANGE-1));
      analogWrite(steeringInB, 0.0);
    }
    else if (PIDoutput < -0.0)
    {
      analogWrite(steeringInA, 0.0);
      analogWrite(steeringInB, (-PIDoutput) * (PWM_RANGE-1));
    }
    else
    {
      analogWrite(steeringInA, 0.0);
      analogWrite(steeringInB, 0.0);
    }
  }
  else
  {
    /* The line is lost; just continue in the same direction */
  }

  /* Dump debug information */

  if (debugPrescaler++ >= 20)
  {
    debugPrescaler = 0;

    LineTracker_DebugInfo();

    Serial.print(" PID:    ");
    Serial.print(PIDoutput);
    Serial.println();

    Serial.println();
  }

  /* Driving */

  HwWrap_MotionFwd();
  delay(20);
  HwWrap_MotionStop();
  delay(80);

  /* delay(100); */
}
