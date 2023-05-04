/* 
    WheelSteering.h
*/

#ifndef __WHEEL_STEERING_H
#define __WHEEL_STEERING_H

#include "Debug.h"

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

        void Set(float steeringSignal);
        void Update(void);

        void DebugInfo(void);
        void DebugInfoGraphics(void);

    protected:

    private:
        float steeringSignalRequested;
        float steeringSignalInUse;
        steeringDirection currentDirection = steeringDirection_STRAIGHT;
        steeringDirection currentWheelPos = steeringDirection_STRAIGHT;
        unsigned timeSlot = 0;
};

#endif
