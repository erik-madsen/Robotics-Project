// A line tracking module based on a row of sensors

#ifndef __LINE_TRACKER_H
#define __LINE_TRACKER_H

#define NO_OF_SENSORS 5

typedef enum
{
  lineState_UNDEFINED,
  lineState_TRACKED,
  lineState_TRACKED_TO_THE_RIGHT,
  lineState_TRACKED_TO_THE_LEFT,
  lineState_LOST_TO_THE_RIGHT,
  lineState_LOST_TO_THE_LEFT,
  lineState_LOST
}
lineState;

class LineTracker
{
  public:
    LineTracker(void);

    void Init(void);
    void Update(lineState *lineState, float *position);

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

    float inputMax;
    float inputMin;

    float mass     = 0.0;
    float torque   = 0.0;
    float centroid = 0.0;

    float positionOfLine = 0.0;

    lineState stateOfTracking  = lineState_UNDEFINED;

};

#endif
