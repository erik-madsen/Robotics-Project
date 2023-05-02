/*
  A HW wrapper for I/O etc.
*/

#define LOGICAL_INPUT_0 0
#define LOGICAL_INPUT_1 1
#define LOGICAL_INPUT_2 2
#define LOGICAL_INPUT_3 3
#define LOGICAL_INPUT_4 4

#define ADC_RANGE 1024
#define PWM_RANGE 256

unsigned HwWrap_AnalogInput(int inputNo);

void HwWrap_DebugString(char *string);
void HwWrap_DebugUnsigned(unsigned value);
void HwWrap_DebugFloat(float value);
void HwWrap_DebugNewLine();
