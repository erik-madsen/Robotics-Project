/* 
    WheelSteering.h
*/

#ifndef __WHEEL_STEERING_H
#define __WHEEL_STEERING_H

#include "Debug.h"

#define steeringOffset_LEFT   (-0.4f)
#define steeringOffset_CENTER ( 0.0f)
#define steeringOffset_RIGHT  ( 0.4f)

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

    protected:

    private:
        float steeringSignalRequested;
        float steeringSignalInUse;
        steeringDirection currentDirection = steeringDirection_STRAIGHT;
        steeringDirection currentWheelPos = steeringDirection_STRAIGHT;
        unsigned timeSlot = 0;
};

#endif
