/* 
    WheelSteering.h
*/

#pragma once

#include "Debug.h"
#include "HwWrap.h"

typedef enum
{
    steeringDirection_STRAIGHT,
    steeringDirection_LEFT,
    steeringDirection_RIGHT
}
steeringDirection;

class WheelSteering
{
    public:
        WheelSteering(void);

        void Set(float steeringRequest);
        void Update(void);

        void DebugInfo(void);
        void DebugInfoGraphics(void);

    protected:

    private:
        HwWrap_SteeringOutput output;

        float steeringSignalRequested;
        float steeringSignalInUse;
        steeringDirection currentDirection = steeringDirection_STRAIGHT;
        steeringDirection currentWheelPos = steeringDirection_STRAIGHT;
        unsigned timeSlot = 0;

};
