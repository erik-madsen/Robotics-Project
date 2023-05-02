// A line tracking module based on a row of sensors

#include "LineTracker.h"
#include "HwWrap.h"

HwWrap HwLine;

LineTracker::LineTracker(void)
{
}

void LineTracker::Calibrate(void)
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

void LineTracker::Init(void)
{
  int i;

  sensor[0].pinNo = LOGICAL_INPUT_0;
  sensor[1].pinNo = LOGICAL_INPUT_1;
  sensor[2].pinNo = LOGICAL_INPUT_2;
  sensor[3].pinNo = LOGICAL_INPUT_3;
  sensor[4].pinNo = LOGICAL_INPUT_4;

  sensor[0].sensorBias = 0.05;
  sensor[1].sensorBias = 0.06;
  sensor[2].sensorBias = 0.12;
  sensor[3].sensorBias = 0.08;
  sensor[4].sensorBias = 0.07;

  sensor[0].sensorScaling = 1.00 / 0.81;
  sensor[1].sensorScaling = 1.00 / 0.81;
  sensor[2].sensorScaling = 1.00 / 0.91;
  sensor[3].sensorScaling = 1.00 / 0.87;
  sensor[4].sensorScaling = 1.00 / 0.84;

  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].inputValue = ((float)HwLine.AnalogInput(sensor[i].pinNo) / ADC_RANGE);
    sensor[i].inputValue -= sensor[i].sensorBias;
    sensor[i].inputValue *= sensor[i].sensorScaling;
  }
  LineTracker::Calibrate();
}

void LineTracker::Update(lineEdgeState *lineState, float *position)
{
  int i;
  bool someHigh = false;
  bool someLow  = false;

  // Read sensors and determine their levels
  
  for (i=0; i<NO_OF_SENSORS; i++)
  {
    sensor[i].inputValue = ((float)HwLine.AnalogInput(sensor[i].pinNo) / ADC_RANGE);
    sensor[i].inputValue -= sensor[i].sensorBias;
    sensor[i].inputValue *= sensor[i].sensorScaling;

    if (sensor[i].inputValue > (commonMax + commonMin) / 2.0)
    {
      sensor[i].logicalLevel = true;
      someHigh = true;
    }
    else
    {
      sensor[i].logicalLevel = false;
      someLow = true;
    }
  }

  // Calculate variance and dispersion
/*   {
    float mean       = 0.0;
    float variance   = 0.0;
    float dispersion = 0.0;

    for (i=0; i<NO_OF_SENSORS; i++)
    {
      mean += sensor[i].inputValue;
    }
    mean /= NO_OF_SENSORS;

    for (i=0; i<NO_OF_SENSORS; i++)
    {
      HwLine.DebugFloat((sensor[i].inputValue - mean));
      HwLine.DebugNewLine();
      variance += (sensor[i].inputValue - mean)*(sensor[i].inputValue - mean)*100;
    }
    variance /= NO_OF_SENSORS;

    HwLine.DebugString(" Mean: ");
    HwLine.DebugFloat(mean);
    HwLine.DebugString(" Variance: ");
    HwLine.DebugFloat(variance);
    HwLine.DebugNewLine();
  }
 */
  // If some sensor levels are high and some are low,
  // then update the references for level detection

  if (someHigh && someLow)
  {
    LineTracker::Calibrate();
  }

  // Search the line's right edge by running 
  // through the sensors from right to left,
  // and set the position value accordingly

  for (i=NO_OF_SENSORS-1; i>=0; i--)
  {
    if (sensor[i].logicalLevel)
    {
      // The right edge is recognized

      if (i==NO_OF_SENSORS-1)
      {
        stateOfRightEdge = lineEdgeState_TRACKED_AT_FAR_RIGHT_SENSOR;
      }
      else if (i==0)
      {
        stateOfRightEdge = lineEdgeState_TRACKED_AT_FAR_LEFT_SENSOR;
      }
      else
      {
        stateOfRightEdge = lineEdgeState_TRACKED;
      }
      positionRelativeToRightEdge = positionValue[i+1];

      break;
    }
    else if (i==0)
    {
      // The right edge is not recognized

      if (stateOfRightEdge == lineEdgeState_LOST_TO_THE_RIGHT ||
          stateOfRightEdge == lineEdgeState_LOST_TO_THE_LEFT  ||
          stateOfRightEdge == lineEdgeState_LOST)
      {
        // Lost state is already recognized
        // Just keep the state and the last position value until the line is found again
      }
      else if (stateOfRightEdge == lineEdgeState_TRACKED_AT_FAR_RIGHT_SENSOR)
      {
        // Before it was at the outmost sensor, so now it is probably beyond there
        stateOfRightEdge = lineEdgeState_LOST_TO_THE_RIGHT;
        positionRelativeToRightEdge = positionValue[NO_OF_SENSORS];
      }
      else if (stateOfRightEdge == lineEdgeState_TRACKED_AT_FAR_LEFT_SENSOR)
      {
        // Before it was at the outmost sensor, so now it is probably beyond there
        stateOfRightEdge = lineEdgeState_LOST_TO_THE_LEFT;
        positionRelativeToRightEdge = positionValue[0];
      }
      else
      {
        // Before it was tracked inside the outmost sensors, so now it is probably gone
        stateOfRightEdge = lineEdgeState_LOST;
      }
    }
  }

  // Search the line's left edge by running 
  // through the sensors from left to right,
  // and set the position value accordingly

  for (i=0; i<=NO_OF_SENSORS-1; i++)
  {
    if (sensor[i].logicalLevel)
    {
      positionRelativeToLeftEdge = positionValue[i];
      break;
    }
    positionRelativeToLeftEdge = positionValue[i+1];
  }

  *position = positionRelativeToRightEdge;
  *lineState = stateOfRightEdge;
}

void LineTracker::DebugInfo(void)
{
  int i;

  HwLine.DebugString(" Max:  ");
  HwLine.DebugFloat(commonMax);
  HwLine.DebugString("  /");

  for (i=0; i<NO_OF_SENSORS; i++)
  {
    if (sensor[i].logicalLevel)
    {
      HwLine.DebugString("  ");
      HwLine.DebugFloat(sensor[i].inputValue);
    }
    else
    {
      HwLine.DebugString("  ....");
    }
  }
  HwLine.DebugNewLine();

  HwLine.DebugString(" Min:  ");
  HwLine.DebugFloat(commonMin);
  HwLine.DebugString("  \\");

  for (i=0; i<NO_OF_SENSORS; i++)
  {
    if (!sensor[i].logicalLevel)
    {
      HwLine.DebugString("  ");
      HwLine.DebugFloat(sensor[i].inputValue);
    }
    else
    {
      HwLine.DebugString("  ....");
    }
  }
  HwLine.DebugNewLine();

  HwLine.DebugString(" Pos:  L ");
  HwLine.DebugFloat(positionRelativeToLeftEdge);
  HwLine.DebugString("  ");
  switch (stateOfRightEdge)
  {
    case lineEdgeState_UNDEFINED:
      HwLine.DebugString("UNDEFINED");
      break;
    case lineEdgeState_TRACKED:
      HwLine.DebugString(" TRACKED ");
      break;
    case lineEdgeState_TRACKED_AT_FAR_RIGHT_SENSOR:
      HwLine.DebugString("TRACKED_R");
      break;
    case lineEdgeState_TRACKED_AT_FAR_LEFT_SENSOR:
      HwLine.DebugString("TRACKED_L");
      break;
    case lineEdgeState_LOST_TO_THE_RIGHT:
      HwLine.DebugString(" LOST_R  ");
      break;
    case lineEdgeState_LOST_TO_THE_LEFT:
      HwLine.DebugString(" LOST_L  ");
      break;
    case lineEdgeState_LOST:
      HwLine.DebugString("  LOST   ");
      break;
  }
  HwLine.DebugString("  R ");
  HwLine.DebugFloat(positionRelativeToRightEdge);
  HwLine.DebugNewLine();
}
