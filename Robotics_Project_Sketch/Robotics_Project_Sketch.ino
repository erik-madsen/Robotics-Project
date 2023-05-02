/*
    An Arduino IDE SW for experimenting with the Off Road Tracer
*/

#include "Debug.h"
#include "HwWrap.h"
#include "LineTracker.h"
#include "WheelSteering.h"
#include "PIDregulator.h"

#ifdef USE_SPEED_CONTROL
static float aiA0;
static float aiA1;

static float aiA0_max = 0.0;
static float aiA0_min = 1.0;
static float aiA0_lim;

static float aiA1_max = 0.0;
static float aiA1_min = 1.0;
static float aiA1_lim;

static unsigned aiA1A0;
static unsigned aiA1A0_last = 999;
#endif /* USE_SPEED_CONTROL */

LineTracker tracker;
WheelSteering steering;
PIDregulator steeringPID;

float steeringSignal = 0.0;
int debugPrescaler = 0;

void setup()
{
    Serial.begin(9600);
    while (!Serial);

    Serial.println();

    tracker.Init();

    analogWrite(steeringInATurnRight, 0.0);
    analogWrite(steeringInBTurnLeft, 0.0);

    steeringPID.SetRangeToIncludeMinusOne(1);
    steeringPID.SetKp(2.5); // 2.5
    steeringPID.SetKi(0.0); // 0.0
    steeringPID.SetKd(0.0); // 2.0
    steeringPID.Init();
}

void loop()
{
    lineState lineTrackedState;
    float trackedPosition;

    /* Control steering using the line tracker and a PID regulator */

#ifdef USE_LINE_TRACKER
    tracker.Update(&lineTrackedState, &trackedPosition);

    switch (lineTrackedState)
    {
        // Stuff to do here could be something like 
        // reversing in the opposite direction 
        // if the line is lost to the right or left
        case lineState_UNDEFINED:
        case lineState_TRACKED:
        case lineState_TRACKED_TO_THE_RIGHT:
        case lineState_TRACKED_TO_THE_LEFT:
        case lineState_LOST_TO_THE_RIGHT:
        case lineState_LOST_TO_THE_LEFT:
        case lineState_LOST:
        {
        }
        break;
    }
#endif /* USE_LINE_TRACKER */

#ifdef USE_STEERING
    steeringSignal = steeringPID.Update(steeringOffset_CENTER - trackedPosition);
    steering.Update(steeringSignal);
#endif /* USE_STEERING */

#ifdef USE_SPEED_CONTROL
    {
        aiA0 = ((float)HwWrap::GetInstance()->AnalogInput(0) / ADC_RANGE);
        if (aiA0 > aiA0_max) aiA0_max = aiA0;
        if (aiA0 < aiA0_min) aiA0_min = aiA0;
        aiA0_lim = (aiA0_max+aiA0_min)/2;

        aiA1 = ((float)HwWrap::GetInstance()->AnalogInput(1) / ADC_RANGE);
        if (aiA1 > aiA1_max) aiA1_max = aiA1;
        if (aiA1 < aiA1_min) aiA1_min = aiA1;
        aiA1_lim = (aiA1_max+aiA1_min)/2;

        aiA1A0 = (aiA1 > (aiA1_lim*10)) + (aiA0 > aiA0_lim);
        if (aiA1A0 != aiA1A0_last)
        {
            aiA1A0_last = aiA1A0;
            HwWrap::GetInstance()->DebugUnsigned( aiA1 > aiA1_lim );
            HwWrap::GetInstance()->DebugUnsigned( aiA0 > aiA0_lim );

            HwWrap::GetInstance()->DebugString("   A1 ");
            HwWrap::GetInstance()->DebugFloat(aiA1_max);
            HwWrap::GetInstance()->DebugString("/");
            HwWrap::GetInstance()->DebugFloat(aiA1_min);
            HwWrap::GetInstance()->DebugString(" ");
            HwWrap::GetInstance()->DebugFloat(aiA1);

            HwWrap::GetInstance()->DebugString("   A2 ");
            HwWrap::GetInstance()->DebugFloat(aiA0_max);
            HwWrap::GetInstance()->DebugString("/");
            HwWrap::GetInstance()->DebugFloat(aiA0_min);
            HwWrap::GetInstance()->DebugString(" ");
            HwWrap::GetInstance()->DebugFloat(aiA0);

            HwWrap::GetInstance()->DebugNewLine();
        }
    }
#endif /* USE_SPEED_CONTROL */


#define DEBUG_MODE 0

    /* Dump debug information */

#if DEBUG_MODE==1
    if (++debugPrescaler >= 20)
#elif DEBUG_MODE==2
    if (++debugPrescaler >= 20)
#elif DEBUG_MODE==3
    if (++debugPrescaler >= 1)
#elif DEBUG_MODE==4
    if (++debugPrescaler >= 50)
#else
    delay(4*1000);
#endif
    {
        debugPrescaler = 0;

#ifdef USE_LINE_TRACKER
        tracker.DebugInfo();
#endif /* USE_LINE_TRACKER */

#ifdef USE_STEERING
        steeringPID.DebugInfo();
        steering.DebugInfo();
#endif /* USE_STEERING */

#if defined USE_LINE_TRACKER || defined USE_STEERING
        HwWrap::GetInstance()->DebugNewLine();
#endif /* USE_LINE_TRACKER || USE_STEERING */
    }

    /* Driving */

#if DEBUG_MODE==1
    HwWrap::GetInstance()->MotionFwd();
    delay(25);
    HwWrap::GetInstance()->MotionStop();
    delay(75);
#elif DEBUG_MODE==2
    delay(100);
#elif DEBUG_MODE==3
    delay(5000);
#elif DEBUG_MODE==4
    analogWrite(motionInADriveBackwards, 0);
    analogWrite(motionInBDriveForwards, 25);
    delay(20);
#endif
}
