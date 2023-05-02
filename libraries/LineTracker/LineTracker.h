// A line tracking module based on a row of sensors

#ifndef __LINE_TRACKER_H
#define __LINE_TRACKER_H

#define NO_OF_SENSORS 5

typedef enum
{
  lineEdgeState_UNDEFINED,
  lineEdgeState_TRACKED,
  lineEdgeState_TRACKED_AT_FAR_RIGHT_SENSOR,
  lineEdgeState_TRACKED_AT_FAR_LEFT_SENSOR,
  lineEdgeState_LOST_TO_THE_RIGHT,
  lineEdgeState_LOST_TO_THE_LEFT,
  lineEdgeState_LOST
}
lineEdgeState;

class LineTracker
{
  public:
    LineTracker(void);

    void Init(void);
    void Update(lineEdgeState *lineState, float *position);

    void DebugInfo(void);

  protected:

  private:
    typedef struct
    {
      unsigned pinNo;
      float    inputValue;
      float    sensorBias;
      float    sensorScaling;
      bool     logicalLevel;
    }
    ts_sensor;

    ts_sensor sensor[NO_OF_SENSORS];

    float commonMax;
    float commonMin;

    float positionRelativeToRightEdge = 0.0;
    float positionRelativeToLeftEdge  = 0.0;
    float positionValue[NO_OF_SENSORS+1] = {0.3, 0.2, 0.1, -0.1, -0.2, -0.3};

    lineEdgeState stateOfRightEdge = lineEdgeState_UNDEFINED;
    lineEdgeState stateOfLeftEdge  = lineEdgeState_UNDEFINED;

    void Calibrate(void);

};

#endif
