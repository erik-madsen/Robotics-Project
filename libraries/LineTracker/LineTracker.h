/* 
    LineTracker.h
*/

#ifndef __LINE_TRACKER_H
#define __LINE_TRACKER_H

#include "Debug.h"
#include "HwWrap.h"

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
        void Update(lineState *lineState, float *veichlePosition);

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

        float mass     = 0.0;
        float torque   = 0.0;
        float centroid = 0.0;

        float positionOfLine = 0.0;

        lineState stateOfTracking  = lineState_UNDEFINED;

#ifdef USE_LINE_TRACKER_SIMULATION
        char simIndicationPoints[(3*NO_OF_SENSORS + 1)];
        unsigned int simIndicationIndex = NO_OF_SENSORS;
        unsigned int simCountingDir = 0;

        void SimulateInputs(void);
#endif
};

#endif
