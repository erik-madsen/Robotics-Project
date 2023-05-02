/*
  A line tracking module based on a row of sensors
*/

#include "Common.h"

#define NO_OF_SENSORS 5

typedef struct
{
  unsigned pinNo;
  float    inputValue;
  float    sensorScaling;
  unsigned logicalLevel;
}
ts_sensor;

void LineTracker_Init();
void LineTracker_Update(t_boolean *lineDetected, float *position);
void LineTracker_DebugInfo();
