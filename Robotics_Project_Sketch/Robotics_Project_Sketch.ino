/*
    An Arduino IDE SW for experimenting with the Off Road Tracer
*/

#include "Debug.h"
#include "Common.h"
#include "HwWrap.h"
#include "LineTracker.h"
#include "WheelSteering.h"
#include "VelocityControl.h"
#include "PIDregulator.h"
#include "SwTimer.h"

static volatile unsigned ISR_velocityTachoInA_level;
static volatile unsigned ISR_velocityTachoInB_level;
static volatile unsigned ISR_velocityTachoQuadratureCounter;
static volatile int ISR_velocityTachoDirection;
static unsigned long ISR_velocityTachoMagnetTime;
static unsigned long ISR_velocityTachoMagnetTimeLast;

#define SYSTEM_TIME_TICK_BASE 50
#define SYSTEM_TIME_ms  1/SYSTEM_TIME_TICK_BASE
#define SYSTEM_TIME_sec 1000/SYSTEM_TIME_TICK_BASE
unsigned long timeStart_SystemTimeTick = 0;
unsigned long timeLimit_SystemTimeTick = SYSTEM_TIME_TICK_BASE;

#define VELOCITY_TACHO_NUMBER_OF_MAGNETS 10
#define VELOCITY_TACHO_QUADRATURES_PER_MAGNET 4

#define VELOCITY_TEST_VELOCITY 0.4
#define VELOCITY_TEST_RAMP 0.02

SwTimer tachoTimer;
#define TACHO_MAX_SPEED_ROTATION_TIME  140 // ~ 429 RPM
#define TACHO_MIN_SPEED_ROTATION_TIME 3160 // ~  19 RPM
#define TACHO_STOPPED_ROTATION_TIME 99999
#define TACHO_TIMEOUT_PERIOD  TACHO_MIN_SPEED_ROTATION_TIME / VELOCITY_TACHO_NUMBER_OF_MAGNETS * 2 * SYSTEM_TIME_ms
static volatile unsigned velocityTachoQuadratureCounter;
static volatile unsigned velocityTachoQuadratureCounterLast;

LineTracker tracker;
PIDregulator trackerPID;
SwTimer trackerTimer;
#define TRACKER_TIMER_ONE_TICK_ONLY 1
#define TRACKER_TIMER_PERIOD  600 * SYSTEM_TIME_ms

WheelSteering steering;
SwTimer steeringTimer;
#define STEERING_TIMER_ONE_TICK_ONLY 1
#define STEERING_TIMER_PERIOD  50 * SYSTEM_TIME_ms
float steeringSignal = 0.0;

VelocityControl velocity;
PIDregulator velocityPID;
SwTimer velocityTimer;
#define VELOCITY_TIMER_ONE_TICK_ONLY 1
#define VELOCITY_TIMER_PERIOD  50 * SYSTEM_TIME_ms
float setPoint;
unsigned long currentRotationTime_ms;
float currentVelocity = 0.0;
float velocitySignal = 0.0;


#ifdef VELOCITY_USE_OUTPUTS
HwWrap_VelocityOutput velocityOutput;
#endif


void ISR_velocityTachoCommon(void)
{
    // Measure the time per revolution of the wheels
    if (ISR_velocityTachoInA_level == 1 && ISR_velocityTachoInB_level == 1)
    {
        unsigned long t = millis();
        ISR_velocityTachoMagnetTime = t - ISR_velocityTachoMagnetTimeLast;
        ISR_velocityTachoMagnetTimeLast = t;
    }
}

void ISR_velocityTachoInA(void)
{
    ISR_velocityTachoInA_level = HwWrap::GetInstance()->DigitalInput(velocityTachoInA);

    if (ISR_velocityTachoInA_level == ISR_velocityTachoInB_level)
    {
        ISR_velocityTachoQuadratureCounter--;
        ISR_velocityTachoDirection = -1;
    }
    else
    {
        ISR_velocityTachoQuadratureCounter++;
        ISR_velocityTachoDirection = 1;
    }

    ISR_velocityTachoCommon();
}

void ISR_velocityTachoInB(void)
{
    ISR_velocityTachoInB_level = HwWrap::GetInstance()->DigitalInput(velocityTachoInB);

    if (ISR_velocityTachoInA_level == ISR_velocityTachoInB_level)
    {
        ISR_velocityTachoQuadratureCounter++;
        ISR_velocityTachoDirection = 1;
    }
    else
    {
        ISR_velocityTachoQuadratureCounter--;
        ISR_velocityTachoDirection = -1;
    }

    ISR_velocityTachoCommon();}


void setup()
{
    pinMode(velocityTachoInA, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(velocityTachoInA), ISR_velocityTachoInA, CHANGE);
    ISR_velocityTachoInA_level = HwWrap::GetInstance()->DigitalInput(velocityTachoInA);

    pinMode(velocityTachoInB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(velocityTachoInB), ISR_velocityTachoInB, CHANGE);
    ISR_velocityTachoInB_level = HwWrap::GetInstance()->DigitalInput(velocityTachoInB);

    analogWrite(steeringInATurnRight, 0);
    analogWrite(steeringInBTurnLeft, 0);
    analogWrite(velocityInADriveBackwards, 0);
    analogWrite(velocityInBDriveForwards, 0);


    velocityPID.SetRangeToIncludeMinusOne(true);
    velocityPID.Init();
    velocityPID.SetKp(0.60);
    velocityPID.SetKi(0.22);
    velocityPID.SetKd(0.56);
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

        tachoTimer.TimerTick();
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

    unsigned velocityTachoQuadratureCounter = ISR_velocityTachoQuadratureCounter;

    if (velocityTachoQuadratureCounter != velocityTachoQuadratureCounterLast)
    {
        velocityTachoQuadratureCounterLast = velocityTachoQuadratureCounter;
        tachoTimer.TimerStart(TACHO_TIMEOUT_PERIOD);

        currentRotationTime_ms = ISR_velocityTachoMagnetTime * VELOCITY_TACHO_NUMBER_OF_MAGNETS;
        currentVelocity = (float)ISR_velocityTachoDirection * (float)TACHO_MAX_SPEED_ROTATION_TIME / (float)currentRotationTime_ms;
    }
    else if (tachoTimer.TimerEvent(TACHO_TIMEOUT_PERIOD) == swTimerEvent_TIMEOUT)
    {
        currentRotationTime_ms = TACHO_STOPPED_ROTATION_TIME;
        currentVelocity = 0.0;
    }


    if (velocityTimer.TimerEvent(VELOCITY_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        setPoint = velocity.Update();
        if (fabs(setPoint) < EPSILON)
        {
            velocityPID.ResetInternalValues();
        }
        velocitySignal = velocityPID.Update(setPoint - currentVelocity);

#ifdef VELOCITY_USE_OUTPUTS
        if      (velocitySignal > EPSILON) velocityOutput.VelocityFwd( velocitySignal);
        else if (velocitySignal < EPSILON) velocityOutput.VelocityBwd(-velocitySignal);
        else velocityOutput.VelocityStop();
#endif

#if defined VELOCITY_USE_DEBUGGING || defined VELOCITY_USE_DEBUGGING_PID
        {
            static unsigned prescaler;
            if (prescaler++ >= 10)
            {
                prescaler = 0;
#ifdef VELOCITY_USE_DEBUGGING
                velocity.DebugInfo();
#endif
#ifdef VELOCITY_USE_DEBUGGING_PID
                velocityPID.DebugInfo();
#endif
            }
        }
#endif
    }
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
