/*
    An Arduino IDE SW for experimenting with the Off Road Tracer
*/

#include "Debug.h"
#include "HwWrap.h"
#include "LineTracker.h"
#include "WheelSteering.h"
#include "PIDregulator.h"
#include "SwTimer.h"

#ifdef MOTION_CONTROL_IN_USE
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
#endif /* MOTION_CONTROL_IN_USE */

LineTracker tracker;
PIDregulator trackerPID;
SwTimer trackerTimer;
#define TRACKER_TIMER_ONE_TICK_ONLY 1
#define TRACKER_TIMER_PERIOD 12

WheelSteering steering;
SwTimer steeringTimer;
#define STEERING_TIMER_ONE_TICK_ONLY 1
#define STEERING_TIMER_PERIOD 1
float steeringSignal = 0.0;

#ifdef MOTION_CONTROL_USE_OUTPUTS
HwWrap_MotionOutput motionOutput;
#endif

void setup()
{
    motionOutput.MotionStop();

    tracker.Init();
    trackerPID.SetRangeToIncludeMinusOne(true);
    trackerPID.SetKp(1.0); // 2.5
    trackerPID.SetKi(0.0); // 0.0
    trackerPID.SetKd(0.0); // 2.0
    trackerPID.Init();
    trackerTimer.TimerStart(TRACKER_TIMER_ONE_TICK_ONLY);

    steeringTimer.TimerStart(STEERING_TIMER_ONE_TICK_ONLY);

    Serial.begin(9600);
    while (!Serial);

    HwWrap::GetInstance()->DebugNewLine();
    HwWrap::GetInstance()->DebugNewLine();
    HwWrap::GetInstance()->DebugString("--------------------  Restarted  --------------------");
    HwWrap::GetInstance()->DebugNewLine();
    HwWrap::GetInstance()->DebugNewLine();
}

void loop()
{
    lineState lineTrackedState;
    float trackedPosition;

#define lineOffset_LEFT   (-0.4f)
#define lineOffset_CENTER ( 0.0f)
#define lineOffset_RIGHT  ( 0.4f)

    float lineOffset = lineOffset_CENTER;

    /* Control steering using the line tracker and a PID regulator */

    trackerTimer.TimerTick();
    if (trackerTimer.TimerEvent(TRACKER_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        tracker.Update(&lineTrackedState, &trackedPosition);
        steeringSignal = trackerPID.Update(lineOffset - trackedPosition);
        steering.Set(steeringSignal);

#ifdef LINE_TRACKER_USE_DEBUGGING
        tracker.DebugInfo();
#ifdef LINE_TRACKER_USE_DEBUGGING_PID
        trackerPID.DebugInfo();
#endif
#endif

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
    }


    steeringTimer.TimerTick();
    if (steeringTimer.TimerEvent(STEERING_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        steering.Update();

#ifdef STEERING_USE_DEBUGGING
        steering.DebugInfo();
#endif
    }


#ifdef MOTION_CONTROL_IN_USE
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
#endif /* MOTION_CONTROL_IN_USE */


#define MOTION_MODE 0

#if MOTION_MODE==1
    motionOutput.MotionFwd();
    delay(25);
    motionOutput.MotionStop();
    delay(75);
#elif MOTION_MODE==2
#ifdef MOTION_CONTROL_USE_OUTPUTS
    analogWrite(motionInADriveBackwards, 0);
    analogWrite(motionInBDriveForwards, 25);
    delay(100);
#endif
#else
    delay(50);
#endif
}
