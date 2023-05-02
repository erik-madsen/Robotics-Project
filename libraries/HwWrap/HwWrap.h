// A HW wrapper class for I/O etc.

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
#define PWM_GAIN  0.7

#define steeringInA  2
#define steeringInB  3
#define motionInA   40
#define motionInB   41

class HwWrap
{
  public:
    HwWrap(void);

    unsigned AnalogInput(int inputNo);

    void MotionStop();
    void MotionFwd();
    void MotionBwd();

    void DebugString(const char *string);
    void DebugUnsigned(unsigned value);
    void DebugFloat(float value);
    void DebugNewLine();

  protected:

  private:

};

#endif
