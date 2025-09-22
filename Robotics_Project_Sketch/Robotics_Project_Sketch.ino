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

#define WHEEL_TACHO_NUMBER_OF_MAGNETS 10
#define WHEEL_TACHO_QUADRATURES_PER_MAGNET 4
#define WHEEL_TACHO_QUADRATURES_PER_REVOLUTION (WHEEL_TACHO_NUMBER_OF_MAGNETS * WHEEL_TACHO_QUADRATURES_PER_MAGNET)
#define WHEEL_TACHO_m_PER_REVOLUTION 0.235

#define VELOCITY_STOP   0.0
#define VELOCITY_MIN    0.2
#define VELOCITY_MEDIUM 0.6
#define VELOCITY_MAX    1.0
#define VELOCITY_STANDARD_RAMP 0.02

SwTimer tachoTimer;
#define TACHO_MAX_SPEED_ROTATION_TIME  140 // ~ 429 RPM
#define TACHO_MIN_SPEED_ROTATION_TIME 3160 // ~  19 RPM
#define TACHO_STOPPED_ROTATION_TIME 99999
#define TACHO_TIMEOUT_PERIOD  TACHO_MIN_SPEED_ROTATION_TIME / WHEEL_TACHO_NUMBER_OF_MAGNETS * 2 * SYSTEM_TIME_ms
static volatile unsigned velocityTachoQuadratureCounter;
static volatile unsigned velocityTachoQuadratureCounterLast;
static unsigned positionReference;

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

SwTimer missionControlTimer;
#define MISSION_CONTROL_TIMER_PERIOD  100 * SYSTEM_TIME_ms
static unsigned missionControlStep = 0;
static unsigned missionControlStepLast = 99;


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
        missionControlTimer.TimerTick();
    }

    lineState lineTrackedState;
    float trackedPosition;

#define lineOffset_LEFT   (-0.4)
#define lineOffset_CENTER ( 0.0)
#define lineOffset_RIGHT  ( 0.4)

    float lineOffset = lineOffset_CENTER;

    /* Velocity control */
    /* ---------------- */

    velocityTachoQuadratureCounter = ISR_velocityTachoQuadratureCounter;

    if (velocityTachoQuadratureCounter != velocityTachoQuadratureCounterLast)
    {
        velocityTachoQuadratureCounterLast = velocityTachoQuadratureCounter;
        tachoTimer.TimerStart(TACHO_TIMEOUT_PERIOD);

        currentRotationTime_ms = ISR_velocityTachoMagnetTime * WHEEL_TACHO_NUMBER_OF_MAGNETS;
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

    /* Steering control using the line tracker and a PID regulator */
    /* ----------------------------------------------------------- */

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

    /* Mission control */
    /* --------------- */

    typedef enum
    {
        missionStep_UNDEFINED,
        missionStep_END,
        missionStep_TIME,
        missionStep_DISTANCE,
    }
    t_mission_type;

    typedef union
    {
        unsigned  time_s;
        float     dist_m;
    }
    t_mission_union;

    typedef struct
    {
        t_mission_type  stepType;
        float           speed;
        float           ramp;
        t_mission_union condition;
    }
    t_mission_step;

    static t_mission_step mission[] =
    {
        { missionStep_TIME,      VELOCITY_STOP,   VELOCITY_STANDARD_RAMP,  {.time_s = 5    } },
        { missionStep_DISTANCE,  VELOCITY_MIN,    VELOCITY_STANDARD_RAMP,  {.dist_m = 1.0f } },

        { missionStep_TIME,      VELOCITY_STOP,   VELOCITY_STANDARD_RAMP,  {.time_s = 5    } },
        { missionStep_DISTANCE,  -VELOCITY_MIN,   VELOCITY_STANDARD_RAMP,  {.dist_m = 1.0f } },

        { missionStep_END,       VELOCITY_STOP,   VELOCITY_STANDARD_RAMP,  {.time_s = 0    } }
    };


    switch (mission[missionControlStep].stepType)
    {
        case missionStep_TIME:
        {
            if (missionControlStep != missionControlStepLast)
            {
                missionControlStepLast = missionControlStep;

                velocity.Set( mission[missionControlStep].speed, mission[missionControlStep].ramp );
                if (mission[missionControlStep].condition.time_s != 0)
                {
                    missionControlTimer.TimerStart( mission[missionControlStep].condition.time_s * SYSTEM_TIME_sec );
                }
                else
                {
                    missionControlTimer.TimerStop();
                }
            }

            if (missionControlTimer.TimerEvent(MISSION_CONTROL_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
            {
                missionControlStep++;
            }
        }
        break;

        case missionStep_DISTANCE:
        {
            if (missionControlStep != missionControlStepLast)
            {
                missionControlStepLast = missionControlStep;

                unsigned distanceInQuadratureCounts = round( (mission[missionControlStep].condition.dist_m / WHEEL_TACHO_m_PER_REVOLUTION) * WHEEL_TACHO_QUADRATURES_PER_REVOLUTION );

                velocity.Set( mission[missionControlStep].speed, mission[missionControlStep].ramp );

                if (mission[missionControlStep].speed > EPSILON)
                {
                    positionReference = velocityTachoQuadratureCounter + distanceInQuadratureCounts;
                }
                else if (mission[missionControlStep].speed < EPSILON)
                {
                    positionReference = velocityTachoQuadratureCounter - distanceInQuadratureCounts;
                }
            }

            if (mission[missionControlStep].speed > EPSILON)
            {
                if (velocityTachoQuadratureCounter >= positionReference)
                {
                    missionControlStep++;
                }
            }
            else if (mission[missionControlStep].speed < EPSILON)
            {
                if (velocityTachoQuadratureCounter <= positionReference)
                {
                    missionControlStep++;
                }
            }
        }
        break;

        case missionStep_END:
        {
            if (missionControlStep != missionControlStepLast)
            {
                velocity.Set( VELOCITY_STOP, VELOCITY_STANDARD_RAMP );
            }
        }
        break;
    }
#endif /* VELOCITY_IN_USE */
}
