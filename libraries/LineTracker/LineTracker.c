/*
  A line tracking module based on a row of sensors
*/

#include "LineTracker.h"
#include "Common.h"
#include "HwWrap.h"

ts_sensor sensor[NO_OF_SENSORS];

float commonMax;
float commonMin;

float positionRelativeToRightEdge = 0.0;
float positionRelativeToLeftEdge  = 0.0;
float positionValue[NO_OF_SENSORS+1] = {0.41, 0.4, 0.2, -0.2, -0.4, -0.41};

void LineTrackerPrivate_Calibrate()
{
  int i;

  commonMax = 0.0;
  commonMin = 1.0;
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    if (sensor[i].inputValue > commonMax) commonMax = sensor[i].inputValue;
    if (sensor[i].inputValue < commonMin) commonMin = sensor[i].inputValue;
  }
}

void LineTracker_Init()
{
  int i;

  sensor[0].pinNo = LOGICAL_INPUT_0;
  sensor[1].pinNo = LOGICAL_INPUT_1;
  sensor[2].pinNo = LOGICAL_INPUT_2;
  sensor[3].pinNo = LOGICAL_INPUT_3;
  sensor[4].pinNo = LOGICAL_INPUT_4;

  sensor[0].sensorScaling = 35.0 / 28.0;
  sensor[1].sensorScaling = 35.0 / 20.0;
  sensor[2].sensorScaling = 35.0 / 37.0;
  sensor[3].sensorScaling = 35.0 / 28.0;
  sensor[4].sensorScaling = 35.0 / 29.0;

  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].inputValue = ((float)HwWrap_AnalogInput(sensor[i].pinNo) / ADC_RANGE) * sensor[i].sensorScaling;
  }
  LineTrackerPrivate_Calibrate();
}

void LineTracker_Update(t_boolean *lineDetected, float *position)
{
  int i;
  t_boolean someHigh = FALSE;
  t_boolean someLow  = FALSE;

  /* Read sensors and determine their levels */
  
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].inputValue = ((float)HwWrap_AnalogInput(sensor[i].pinNo) / ADC_RANGE) * sensor[i].sensorScaling;

    if (sensor[i].inputValue > (commonMax + commonMin) / 2.0)
    {
      sensor[i].logicalLevel = 1;
      someHigh = TRUE;
    }
    else
    {
      sensor[i].logicalLevel = 0;
      someLow = TRUE;
    }
  }

  if (someHigh && someLow)
  {
    LineTrackerPrivate_Calibrate();
  }

  /* Determine the position of the line's right edge and set the position value */

  for (i=NO_OF_SENSORS-1; i>=0; i--)
  {
    positionRelativeToRightEdge = positionValue[i+1];
    if (sensor[i].logicalLevel == 1)
    {
      break;
    }
    positionRelativeToRightEdge = positionValue[i];
  }

  /* Determine the position of the line's left edge and set the position value */

  for (i=0; i<=NO_OF_SENSORS-1; i++)
  {
    positionRelativeToLeftEdge = positionValue[i];
    if (sensor[i].logicalLevel == 1)
    {
      break;
    }
    positionRelativeToLeftEdge = positionValue[i+1];
  }

  *position = positionRelativeToRightEdge;
  *lineDetected = (positionRelativeToRightEdge < positionRelativeToLeftEdge);
}

void LineTracker_DebugInfo()
{
  int i;

  HwWrap_DebugString(" Max:    ");
  HwWrap_DebugFloat(commonMax);
  HwWrap_DebugNewLine();

  HwWrap_DebugString(" Input:  ");
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    HwWrap_DebugFloat(sensor[i].inputValue);
    HwWrap_DebugString(":");
    HwWrap_DebugUnsigned(sensor[i].logicalLevel);
    HwWrap_DebugString("  ");
  }
  HwWrap_DebugNewLine();

  HwWrap_DebugString(" Min:    ");
  HwWrap_DebugFloat(commonMin);
  HwWrap_DebugNewLine();

  HwWrap_DebugString(" Pos:    ");
  HwWrap_DebugFloat(positionRelativeToLeftEdge);
  HwWrap_DebugString("  ");
  HwWrap_DebugFloat(positionRelativeToRightEdge);
  HwWrap_DebugNewLine();
}
