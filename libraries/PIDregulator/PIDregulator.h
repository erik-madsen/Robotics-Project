/* 
    PIDregulator.cpp
    The file contains the definition of a PID regulator.
*/

#ifndef __PID_REGULATOR_H
#define __PID_REGULATOR_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float pTerm;
    float iTerm;
    float dTerm;

    float errorSum;
}
t_PID_parameters;

class PIDregulator
{
    public:
        PIDregulator(void);

        void  ResetInternalValues(void);
        void  Init(void);
        float Update(float error);  // Update the PID regulator based on the current error

        void  SetRangeToIncludeMinusOne(bool setting);
        void  SetKp(float value);
        void  SetKi(float value);
        void  SetKd(float value);
        void  GetParams(t_PID_parameters* param_ptr);

        void DebugInfo(void);

    protected:

    private:
        bool enabled;
        bool rangeToMinusOne;

        float Kp;
        float Ki;
        float Kd;

        float pTerm;
        float iTerm;
        float dTerm;

        float errorSum;
        float errorSumMax;
        float errorLast;
        float output;
};

#endif
