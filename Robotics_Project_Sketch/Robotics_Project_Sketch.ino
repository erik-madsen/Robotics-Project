/*
    An Arduino IDE SW for experimenting with the Off Road Tracer
*/

#include "Debug.h"
#include "HwWrap.h"
#include "LineTracker.h"
#include "WheelSteering.h"
#include "VelocityControl.h"
#include "PIDregulator.h"
#include "SwTimer.h"

static unsigned ISR_velocityTachoInA_level;
static unsigned ISR_velocityTachoInB_level;
static unsigned ISR_velocityTachoCounter;
static unsigned velocityTachoCounter;
static unsigned velocityTachoCounter_last = velocityTachoCounter;

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

VelocityControl velocity;
PIDregulator velocityPID;
SwTimer velocityTimer;
#define VELOCITY_TIMER_ONE_TICK_ONLY 1
#define VELOCITY_TIMER_PERIOD 2
float currentVelocity = 0.0;
float velocitySignal = 0.0;

#define VELOCITY_TEST_VELOCITY 0.5
#define VELOCITY_TEST_RAMP 0.01
#define VELOCITY_TEST_COUNT_IN_EACH_DIRECTION 100

#ifdef VELOCITY_USE_OUTPUTS
HwWrap_VelocityOutput velocityOutput;
#endif

unsigned long timeStart_SystemTimeTick = 0;
unsigned long timeLimit_SystemTimeTick = 50;


void ISR_velocityTachoInA(void)
{
    ISR_velocityTachoInA_level = HwWrap::GetInstance()->DigitalInput(velocityTachoInA);

    if (ISR_velocityTachoInA_level == ISR_velocityTachoInB_level)
    {
        ISR_velocityTachoCounter--;
    }
    else
    {
        ISR_velocityTachoCounter++;
    }
}

void ISR_velocityTachoInB(void)
{
    ISR_velocityTachoInB_level = HwWrap::GetInstance()->DigitalInput(velocityTachoInB);

    if (ISR_velocityTachoInA_level == ISR_velocityTachoInB_level)
    {
        ISR_velocityTachoCounter++;
    }
    else
    {
        ISR_velocityTachoCounter--;
    }
}


void setup()
{
    pinMode(velocityTachoInA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(velocityTachoInA), ISR_velocityTachoInA, CHANGE);
    pinMode(velocityTachoInB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(velocityTachoInB), ISR_velocityTachoInB, CHANGE);

    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, 0);
    analogWrite(velocityInADriveBackwards, 0);
    analogWrite(velocityInBDriveForwards, 0);


    velocityPID.SetRangeToIncludeMinusOne(true);
    velocityPID.Init();
    velocityPID.SetKp(0.4);
    velocityPID.SetKi(0.6);
    velocityPID.SetKd(0.02);
    velocityTimer.TimerStart(VELOCITY_TIMER_ONE_TICK_ONLY);

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

    velocity.Set(VELOCITY_TEST_VELOCITY, VELOCITY_TEST_RAMP);
}

void loop()
{
    if (millis() - timeStart_SystemTimeTick > timeLimit_SystemTimeTick)
    {
        timeStart_SystemTimeTick = millis();

        velocityTimer.TimerTick();
        trackerTimer.TimerTick();
        steeringTimer.TimerTick();
    }

    lineState lineTrackedState;
    float trackedPosition;

#define lineOffset_LEFT   (-0.4)
#define lineOffset_CENTER ( 0.0)
#define lineOffset_RIGHT  ( 0.4)

    float lineOffset = lineOffset_CENTER;

    /* Control velocity */

    if (velocityTimer.TimerEvent(VELOCITY_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        float setPoint = velocity.Update();
        velocitySignal = velocityPID.Update(setPoint - currentVelocity);

#ifdef VELOCITY_USE_OUTPUTS
        if      (velocitySignal > 0.0) velocityOutput.VelocityFwd( velocitySignal);
        else if (velocitySignal < 0.0) velocityOutput.VelocityBwd(-velocitySignal);
        else velocityOutput.VelocityStop();
#endif

#ifdef VELOCITY_USE_DEBUGGING
        velocity.DebugInfo();
#ifdef VELOCITY_USE_DEBUGGING_PID
        velocityPID.DebugInfo();
#endif
#endif
    /* Control steering using the line tracker and a PID regulator */

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


    if (steeringTimer.TimerEvent(STEERING_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        steering.Update();

#ifdef STEERING_USE_DEBUGGING
        steering.DebugInfo();
#endif
    }


#ifdef VELOCITY_IN_USE
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
#endif /* VELOCITY_IN_USE */
}
