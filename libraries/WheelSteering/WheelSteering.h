/* 
    WheelSteering.h
    A module for controlling the steering wheels
*/

#ifndef __WHEEL_STEERING_H
#define __WHEEL_STEERING_H

#include "Debug.h"

#define steeringOffset_LEFT   (-0.4f)
#define steeringOffset_CENTER ( 0.0f)
#define steeringOffset_RIGHT  ( 0.4f)

typedef enum
{
    steeringDirection_LEFT,
    steeringDirection_STRAIGHT,
    steeringDirection_RIGHT
}
steeringDirection;

class WheelSteering
{
    public:
        WheelSteering(void);

        void Update(float steeringSignal);

        void DebugInfo(void);

    protected:

    private:
        steeringDirection currentDir;
};

#endif
