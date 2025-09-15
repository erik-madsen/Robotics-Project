/*
    SwTimer.cpp

    Responsibility:
    ---------------
    Implement a SW timer class.
*/

#include "Debug.h"
#include "SwTimer.h"

SwTimer::SwTimer
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    TimerStop();
}


void SwTimer::TimerStart
//  --------------------------------------------------------------------------------
(
    unsigned timeValue
)
//  --------------------------------------------------------------------------------
{
    timer = timeValue;
}

void SwTimer::TimerStop
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    timer = swTimerValue_STOPPED;
}

void SwTimer::TimerTick
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    if (timer != swTimerValue_STOPPED)
    {
        if (timer > swTimerValue_DONE)
        {
            timer--;
        }
    }
}

swTimerEvent_t SwTimer::TimerEvent
//  --------------------------------------------------------------------------------
(
    unsigned reloadValue
)
//  --------------------------------------------------------------------------------
{
    swTimerEvent_t retVal = swTimerEvent_UNDEFINED;
    
    if (timer == swTimerValue_DONE)
    {
        timer = reloadValue;
        retVal = swTimerEvent_TIMEOUT;
    }

    return retVal;
}
