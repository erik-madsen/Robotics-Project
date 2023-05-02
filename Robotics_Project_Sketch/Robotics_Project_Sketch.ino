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


#ifdef USE_LINE_TRACKER
LineTracker tracker;
#define PRESCALER_TRACKER_COUNT 12
unsigned prescalerTracker = PRESCALER_TRACKER_COUNT;
#endif


#ifdef USE_STEERING
WheelSteering steering;
PIDregulator steeringPID;
#define PRESCALER_STEERING_COUNT 1
unsigned prescalerSteering = PRESCALER_STEERING_COUNT;
#endif

float steeringSignal = 0.0;

void setup()
{
    Serial.begin(9600);
    while (!Serial);

    HwWrap::GetInstance()->SteeringStraight();

    tracker.Init();

    steeringPID.SetRangeToIncludeMinusOne(1);
    steeringPID.SetKp(1.0); // 2.5
    steeringPID.SetKi(0.0); // 0.0
    steeringPID.SetKd(0.0); // 2.0
    steeringPID.Init();

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

    /* Control steering using the line tracker and a PID regulator */

#ifdef USE_LINE_TRACKER
    if (prescalerTracker++ >= (PRESCALER_TRACKER_COUNT-1))
    {
        prescalerTracker = 0;

        tracker.Update(&lineTrackedState, &trackedPosition);

        tracker.DebugInfo();

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

        steeringSignal = steeringPID.Update(steeringOffset_CENTER - trackedPosition);
        steering.Set(steeringSignal);
    }
#endif /* USE_LINE_TRACKER */


#ifdef USE_STEERING
    if (prescalerSteering++ >= (PRESCALER_STEERING_COUNT-1))
    {
        prescalerSteering = 0;

        steering.Update();

        // steeringPID.DebugInfo();
        steering.DebugInfo();
    }
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
#else
    delay(500);
#endif
}
