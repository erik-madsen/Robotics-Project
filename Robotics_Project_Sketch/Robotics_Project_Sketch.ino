/*
  An Arduino IDE SW for experimenting with the line tracking module controlling the steering
*/

#include "HwWrap.h"
#include "LineTracker.h"
#include "PIDregulator.h"

LineTracker tracker;
PIDregulator PID;
HwWrap HwApp;

float PIDoutput;
int debugPrescaler = 0;

void setup()
{
  Serial.begin(9600);
  while (!Serial);

  tracker.Init();

  PID.SetRangeToIncludeMinusOne(1);
  PID.SetKp(3.0); // 2.6
  PID.SetKi(0.0); // 0.0
  PID.SetKd(1.0); // 6.0
  PID.Init();
}

void loop()
{
  lineEdgeState lineState;
  float position;

  /* Control steering using the line tracker and a PID regulator */

  tracker.Update(&lineState, &position);

  switch (lineState)
  {
    case lineEdgeState_TRACKED:
    case lineEdgeState_TRACKED_AT_FAR_RIGHT_SENSOR:
    case lineEdgeState_TRACKED_AT_FAR_LEFT_SENSOR:
    case lineEdgeState_LOST_TO_THE_RIGHT:
    case lineEdgeState_LOST_TO_THE_LEFT:
    {
    }
    break;

    case lineEdgeState_LOST:
    case lineEdgeState_UNDEFINED:
    {
      position = 0.0;
    }

  }

  PIDoutput = PID.Update(0.0 - position);

  if (PIDoutput > 0.0)
  {
    analogWrite(steeringInA, PIDoutput * PWM_GAIN * (PWM_RANGE-1));
    analogWrite(steeringInB, 0.0);
  }
  else if (PIDoutput < -0.0)
  {
    analogWrite(steeringInA, 0.0);
    analogWrite(steeringInB, (-PIDoutput) * PWM_GAIN * (PWM_RANGE-1));
  }
  else
  {
    analogWrite(steeringInA, 0.0);
    analogWrite(steeringInB, 0.0);
  }


#define DEBUG_MODE 1

  /* Dump debug information */

#if DEBUG_MODE==1
  if (++debugPrescaler >= 20)
#elif DEBUG_MODE==2
  if (++debugPrescaler >= 20)
#else
  if (++debugPrescaler >= 1)
#endif
  {
    debugPrescaler = 0;
    tracker.DebugInfo();
    PID.DebugInfo();
    Serial.println();
  }

  /* Driving */

#if DEBUG_MODE==1
  HwApp.MotionFwd();
  delay(15);
  HwApp.MotionStop();
  delay(85);
#elif DEBUG_MODE==2
  delay(100);
#else
  delay(2000);
#endif
}
