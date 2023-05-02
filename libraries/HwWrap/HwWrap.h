/* 
    HwWrap.h
    A HW wrapper class for I/O etc.
    The class uses the singleton pattern.
*/

#include "stdint.h"

#ifndef __HW_WRAP_H
#define __HW_WRAP_H

#define NO_OF_SENSORS 5

#define LOGICAL_INPUT_0 0
#define LOGICAL_INPUT_1 1
#define LOGICAL_INPUT_2 2
#define LOGICAL_INPUT_3 3
#define LOGICAL_INPUT_4 4

#define ADC_RANGE 1024
#define PWM_RANGE 256
#define PWM_GAIN  1.0

#define steeringInATurnRight    7
#define steeringInBTurnLeft     6
#define motionInADriveBackwards 4
#define motionInBDriveForwards  5

class HwWrap
{
    public:
        static HwWrap* GetInstance() { return my_instance; };

        void Init(void);

        unsigned AnalogInput(uint8_t inputNo);
        void AnalogOutput(uint8_t outputNo, unsigned int value);

        unsigned DigitalInput(uint8_t inputNo);

        void MotionStop();
        void MotionFwd();
        void MotionBwd();

        void DebugString(char *string);
        void DebugUnsigned(unsigned value);
        void DebugFloat(float value);
        void DebugNewLine(void);

    protected:

    private:
        HwWrap(void);
        static HwWrap* my_instance;

};

#endif
