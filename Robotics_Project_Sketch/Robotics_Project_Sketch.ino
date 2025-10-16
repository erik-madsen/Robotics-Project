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

#define SYSTEM_TIME_TICK_BASE 10
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
#define VELOCITY_STANDARD_RAMP 0.04
#define VELOCITY_NO_RAMP       1.0
#define VELOCITY_DIRECT_OUTPUT 0.30

SwTimer tachoTimer;
#define TACHO_MAX_SPEED_ROTATION_TIME  140 // ~ 429 RPM
#define TACHO_MIN_SPEED_ROTATION_TIME 1100 // ~  55 RPM
#define TACHO_STOPPED_ROTATION_TIME 99999
#define TACHO_TIMEOUT_PERIOD  TACHO_MIN_SPEED_ROTATION_TIME / WHEEL_TACHO_NUMBER_OF_MAGNETS * 2 * SYSTEM_TIME_ms
static volatile unsigned velocityTachoQuadratureCounter;
static volatile unsigned velocityTachoQuadratureCounterLast;
static unsigned positionReference;

LineTracker tracker;
PIDregulator trackerPID;
#define TRACKER_PID_KP 1.00  // 2.5
#define TRACKER_PID_KI 0.00  // 0.0
#define TRACKER_PID_KD 0.00  // 2.0
SwTimer trackerTimer;
#define TRACKER_TIMER_ONE_TICK_ONLY 1
#define TRACKER_TIMER_PERIOD  600 * SYSTEM_TIME_ms

WheelSteering steering;
SwTimer steeringTimer;
#define STEERING_TIMER_ONE_TICK_ONLY 1
#define STEERING_TIMER_PERIOD  50 * SYSTEM_TIME_ms
float steeringSignal = 0.0;

VelocityControl velocity;
SwTimer velocityTimer;
#define VELOCITY_TIMER_ONE_TICK_ONLY 1
#define VELOCITY_TIMER_PERIOD  20 * SYSTEM_TIME_ms
bool velocityControlEnabled;
float velocitySetPoint;
unsigned long currentRotationTime_ms;
float currentVelocity = 0.0;

PIDregulator velocityPID;
#define VELOCITY_PID_KP 0.50
#define VELOCITY_PID_KI 0.17
#define VELOCITY_PID_KD 0.00
float velocitySignal = 0.0;
SwTimer velocityPidTimer;
#define VELOCITY_PID_TIMER_ONE_TICK_ONLY 1
#define VELOCITY_PID_TIMER_PERIOD  60 * SYSTEM_TIME_ms

#ifdef USE_STATUS_OVERVIEW
SwTimer statusOverviewTimer;
#define STATUS_OVERVIEW_TIMER_ONE_TICK_ONLY 1
#define STATUS_OVERVIEW_TIMER_PERIOD  1 * SYSTEM_TIME_sec
bool statusOverviewEnabled;
#endif

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
    velocityPID.SetKp(VELOCITY_PID_KP);
    velocityPID.SetKi(VELOCITY_PID_KI);
    velocityPID.SetKd(VELOCITY_PID_KD);
    velocityPidTimer.TimerStart(VELOCITY_PID_TIMER_ONE_TICK_ONLY);

    velocityTimer.TimerStart(VELOCITY_TIMER_ONE_TICK_ONLY);

    tracker.Init();
    trackerPID.SetRangeToIncludeMinusOne(true);
    trackerPID.SetKp(TRACKER_PID_KP);
    trackerPID.SetKi(TRACKER_PID_KI);
    trackerPID.SetKd(TRACKER_PID_KD);
    trackerPID.Init();
    trackerTimer.TimerStart(TRACKER_TIMER_ONE_TICK_ONLY);

    steeringTimer.TimerStart(STEERING_TIMER_ONE_TICK_ONLY);

#ifdef USE_STATUS_OVERVIEW
    statusOverviewTimer.TimerStart(STATUS_OVERVIEW_TIMER_ONE_TICK_ONLY);
#endif


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
        velocityPidTimer.TimerTick();
        trackerTimer.TimerTick();
        steeringTimer.TimerTick();
        missionControlTimer.TimerTick();

#ifdef USE_STATUS_OVERVIEW
        statusOverviewTimer.TimerTick();
#endif
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

    // Determine the current velocity

    if (velocityTachoQuadratureCounter != velocityTachoQuadratureCounterLast)
    {
        velocityTachoQuadratureCounterLast = velocityTachoQuadratureCounter;
        tachoTimer.TimerStart(TACHO_TIMEOUT_PERIOD);

        currentRotationTime_ms = ISR_velocityTachoMagnetTime * WHEEL_TACHO_NUMBER_OF_MAGNETS;
        if (currentRotationTime_ms != 0)
        {
            currentVelocity = (float)ISR_velocityTachoDirection * (float)TACHO_MAX_SPEED_ROTATION_TIME / (float)currentRotationTime_ms;
        }
        else
        {
            currentRotationTime_ms = TACHO_STOPPED_ROTATION_TIME;
            currentVelocity = 0.0;
        }
    }
    else if (tachoTimer.TimerEvent(TACHO_TIMEOUT_PERIOD) == swTimerEvent_TIMEOUT)
    {
        currentRotationTime_ms = TACHO_STOPPED_ROTATION_TIME;
        currentVelocity = 0.0;
    }

    // Determine the requested velocity

    if (velocityTimer.TimerEvent(VELOCITY_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        velocitySetPoint = velocity.Update();

        if (EPSILON < velocitySetPoint && velocitySetPoint < VELOCITY_MIN)
        {
            velocitySetPoint = VELOCITY_MIN;
        }
        if (-VELOCITY_MIN < velocitySetPoint && velocitySetPoint < -EPSILON)
        {
            velocitySetPoint = -VELOCITY_MIN;
        }
    }

    // Control the velocity

    if (velocityControlEnabled)
    {
        if (velocityPidTimer.TimerEvent(VELOCITY_PID_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
        {
            if (fabs(velocitySetPoint) < EPSILON)
            {
                velocityPID.ResetInternalValues();
            }
            velocitySignal = velocityPID.Update(velocitySetPoint - currentVelocity);

#ifdef VELOCITY_USE_OUTPUTS
            if      (velocitySignal >  EPSILON) velocityOutput.VelocityFwd( fabs(velocitySignal) );
            else if (velocitySignal < -EPSILON) velocityOutput.VelocityBwd( fabs(velocitySignal) );
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
        missionStep_VELOCITY_DIRECT_OUTPUT,
        missionStep_PAUSE,
        missionStep_TIME,
        missionStep_DISTANCE,
    }
    t_mission_type;

    typedef union
    {
        unsigned  dummy;
        float     speed;
        float     output;
    }
    t_mission_param_1;

    typedef union
    {
        unsigned  dummy;
        float     ramp;
    }
    t_mission_param_2;

    typedef union
    {
        unsigned  dummy;
        unsigned  time_s;
        float     dist_m;
    }
    t_mission_param_3;

    typedef struct
    {
        t_mission_type    stepType;
        t_mission_param_1 param_1;
        t_mission_param_2 param_2;
        t_mission_param_3 param_3;
    }
    t_mission_step;

    static t_mission_step mission[] =
    {
        { missionStep_PAUSE,                    { .speed  =  VELOCITY_STOP          },   { .ramp  =  VELOCITY_NO_RAMP      },   { .time_s = 3    }  },
        { missionStep_VELOCITY_DIRECT_OUTPUT,   { .output =  VELOCITY_DIRECT_OUTPUT },   { .dummy =  0                     },   { .time_s = 2    }  },
        { missionStep_PAUSE,                    { .speed  =  VELOCITY_STOP          },   { .ramp  =  VELOCITY_NO_RAMP      },   { .time_s = 3    }  },
        { missionStep_VELOCITY_DIRECT_OUTPUT,   { .output = -VELOCITY_DIRECT_OUTPUT },   { .dummy =  0                     },   { .time_s = 2    }  },

        { missionStep_PAUSE,                    { .speed  =   VELOCITY_STOP         },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 3    }  },
        { missionStep_DISTANCE,                 { .speed  =   VELOCITY_MIN          },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .dist_m = 1.5f }  },
        { missionStep_PAUSE,                    { .speed  =   VELOCITY_STOP         },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 3    }  },
        { missionStep_DISTANCE,                 { .speed  =  -VELOCITY_MIN          },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .dist_m = 1.5f }  },

        { missionStep_PAUSE,                    { .speed  =   VELOCITY_STOP         },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 3    }  },
        { missionStep_DISTANCE,                 { .speed  =   VELOCITY_MEDIUM       },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .dist_m = 1.5f }  },
        { missionStep_PAUSE,                    { .speed  =   VELOCITY_STOP         },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 3    }  },
        { missionStep_DISTANCE,                 { .speed  =  -VELOCITY_MEDIUM       },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .dist_m = 1.5f }  },

        { missionStep_PAUSE,                    { .speed  =   VELOCITY_STOP         },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 3    }  },
        { missionStep_TIME,                     { .speed  =   VELOCITY_MAX          },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 1    }  },
        { missionStep_PAUSE,                    { .speed  =   VELOCITY_STOP         },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 3    }  },
        { missionStep_TIME,                     { .speed  =  -VELOCITY_MAX          },   { .ramp  = VELOCITY_STANDARD_RAMP },   { .time_s = 1    }  },

        { missionStep_END,                      { .dummy  =  0                      },   { .dummy = 0                      },   { .dummy  = 0    }  }
    };


    switch (mission[missionControlStep].stepType)
    {
        case missionStep_VELOCITY_DIRECT_OUTPUT:
        {
            if (missionControlStep != missionControlStepLast)
            {
                missionControlStepLast = missionControlStep;

                // Mission step entry

                velocityControlEnabled = FALSE;
#ifdef USE_STATUS_OVERVIEW
                statusOverviewEnabled = true;
#endif
#ifdef VELOCITY_USE_OUTPUTS
                if      (mission[missionControlStep].param_1.output >  EPSILON) velocityOutput.VelocityFwd( fabs(mission[missionControlStep].param_1.output) );
                else if (mission[missionControlStep].param_1.output < -EPSILON) velocityOutput.VelocityBwd( fabs(mission[missionControlStep].param_1.output) );
                else velocityOutput.VelocityStop();
#endif
                missionControlTimer.TimerStart( mission[missionControlStep].param_3.time_s * SYSTEM_TIME_sec );
            }

            if (missionControlTimer.TimerEvent(MISSION_CONTROL_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
            {
                missionControlTimer.TimerStop();
                missionControlStep++;
            }
        }
        break;

        case missionStep_PAUSE:
        {
            if (missionControlStep != missionControlStepLast)
            {
                missionControlStepLast = missionControlStep;

                // Mission step entry

                velocityControlEnabled = true;
#ifdef USE_STATUS_OVERVIEW
                statusOverviewEnabled = true;
#endif

                velocity.Set( mission[missionControlStep].param_1.speed, mission[missionControlStep].param_2.ramp );
                missionControlTimer.TimerStart( mission[missionControlStep].param_3.time_s * SYSTEM_TIME_sec );
            }

            if (missionControlTimer.TimerEvent(MISSION_CONTROL_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
            {
                missionControlTimer.TimerStop();
                missionControlStep++;
            }
        }
        break;

        case missionStep_TIME:
        {
            if (missionControlStep != missionControlStepLast)
            {
                missionControlStepLast = missionControlStep;

                // Mission step entry

                velocityControlEnabled = true;
#ifdef USE_STATUS_OVERVIEW
                statusOverviewEnabled = true;
#endif

                velocity.Set( mission[missionControlStep].param_1.speed, mission[missionControlStep].param_2.ramp );
                missionControlTimer.TimerStart( mission[missionControlStep].param_3.time_s * SYSTEM_TIME_sec );
            }

            if (missionControlTimer.TimerEvent(MISSION_CONTROL_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
            {
                missionControlTimer.TimerStop();
                missionControlStep++;
            }
        }
        break;

        case missionStep_DISTANCE:
        {
            if (missionControlStep != missionControlStepLast)
            {
                missionControlStepLast = missionControlStep;

                // Mission step entry

                velocityControlEnabled = true;
#ifdef USE_STATUS_OVERVIEW
                statusOverviewEnabled = true;
#endif

                unsigned distanceInQuadratureCounts = round( (mission[missionControlStep].param_3.dist_m / WHEEL_TACHO_m_PER_REVOLUTION) * WHEEL_TACHO_QUADRATURES_PER_REVOLUTION );

                velocity.Set( mission[missionControlStep].param_1.speed, mission[missionControlStep].param_2.ramp );

                if (mission[missionControlStep].param_1.speed > EPSILON)
                {
                    positionReference = velocityTachoQuadratureCounter + distanceInQuadratureCounts;
                }
                else if (mission[missionControlStep].param_1.speed < EPSILON)
                {
                    positionReference = velocityTachoQuadratureCounter - distanceInQuadratureCounts;
                }
            }

            if (mission[missionControlStep].param_1.speed > EPSILON)
            {
                if (velocityTachoQuadratureCounter >= positionReference)
                {
                    missionControlStep++;
                }
            }
            else if (mission[missionControlStep].param_1.speed < EPSILON)
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
#ifdef USE_STATUS_OVERVIEW
                statusOverviewEnabled = false;
#endif
                if (velocityControlEnabled)
                {
                    velocity.Set( VELOCITY_STOP, VELOCITY_STANDARD_RAMP );
                }
                else
                {
                    velocityOutput.VelocityStop();
                }
            }
        }
        break;
    }

    /* Miscellaneous */
    /* ------------- */

#ifdef USE_STATUS_OVERVIEW
    if (statusOverviewEnabled)
    if (statusOverviewTimer.TimerEvent(STATUS_OVERVIEW_TIMER_PERIOD) == swTimerEvent_TIMEOUT)
    {
        unsigned velocityQuadratureCounter;
        static unsigned header_prescaler = 2;

        velocityQuadratureCounter = ISR_velocityTachoQuadratureCounter;

        if (header_prescaler-- == 0)
        {
            header_prescaler = 10;
            HwWrap::GetInstance()->DebugString("step \t setp. \t signal \t [ms] ~ RPM \t \t ref. \t pos.");
            HwWrap::GetInstance()->DebugNewLine();
        }

        HwWrap::GetInstance()->DebugUnsigned(missionControlStep);
        HwWrap::GetInstance()->DebugString(" \t ");

        // Regulation information

        HwWrap::GetInstance()->DebugFloat(velocitySetPoint);
        HwWrap::GetInstance()->DebugString("\t ");
        HwWrap::GetInstance()->DebugFloat(velocitySignal);
        HwWrap::GetInstance()->DebugString("\t ");
        HwWrap::GetInstance()->DebugString("\t ");

        // Speed information

        if (currentRotationTime_ms == TACHO_STOPPED_ROTATION_TIME)
        {
            HwWrap::GetInstance()->DebugString("---");
        }
        else
        {
            HwWrap::GetInstance()->DebugUnsigned(currentRotationTime_ms);
        }
        HwWrap::GetInstance()->DebugString(" \t");
        HwWrap::GetInstance()->DebugUnsigned((unsigned long)60*1000/currentRotationTime_ms);  // RPM
        HwWrap::GetInstance()->DebugString(" \t ");
        HwWrap::GetInstance()->DebugString(" \t ");

        // Position information

        HwWrap::GetInstance()->DebugUnsigned(positionReference);
        HwWrap::GetInstance()->DebugString(" \t ");
        HwWrap::GetInstance()->DebugUnsigned(velocityTachoQuadratureCounter);
        HwWrap::GetInstance()->DebugNewLine();
    }
#endif /* USE_STATUS_OVERVIEW */
}
