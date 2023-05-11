/* 
    HwWrap.h
*/

#pragma once

#include "stdint.h"

#define NO_OF_SENSORS 5

#define ANALOG_INPUT_0 0
#define ANALOG_INPUT_1 1
#define ANALOG_INPUT_2 2
#define ANALOG_INPUT_3 3
#define ANALOG_INPUT_4 4

#define ADC_RANGE 1024
#define PWM_RANGE 256

#define steeringInATurnRight    7
#define steeringInBTurnLeft     6
#define motionInADriveBackwards 4
#define motionInBDriveForwards  5

class HwWrap
{
    public:
        static HwWrap* GetInstance() { return my_instance; };

        unsigned AnalogInput(uint8_t inputNo);
        void AnalogOutput(uint8_t outputNo, unsigned int value);

        unsigned DigitalInput(uint8_t inputNo);

        void DebugString(const char *string);
        void DebugUnsigned(unsigned value);
        void DebugFloat(float value);
        void DebugNewLine(void);

    protected:

    private:
        HwWrap(void);
        static HwWrap* my_instance;

};

class HwWrap_MotionOutput
{
    public:
        void MotionStop(void);
        void MotionFwd(void);
        void MotionBwd(void);

};

class HwWrap_SteeringOutput
{
    public:
        void SteeringStraight(void);
        void SteeringRight(void);
        void SteeringLeft(void);

};
