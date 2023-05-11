/* 
    SwTimer.h
*/

#pragma once

typedef enum
{
    swTimerEvent_UNDEFINED,
    swTimerEvent_TIMEOUT
}
swTimerEvent_t;

class SwTimer
{
    public:
        SwTimer(void);

        void TimerStart(unsigned timeValue);
        void TimerStop(void);

        void TimerTick(void);
        swTimerEvent_t TimerEvent(unsigned reloadValue);

    protected:

    private:
        enum swTimerValue_e
        {
            swTimerValue_STOPPED = -1,
            swTimerValue_DONE   =  0
        };

        int timer;

};
