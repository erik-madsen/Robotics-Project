/* 
    VelocityControl.h
*/

#pragma once

#include "Debug.h"
#include "HwWrap.h"

class VelocityControl
{
    public:
        VelocityControl(void);

        void Set(float velocityRequest, float velocityRamp);
        float Update(void);

        void DebugInfo(void);

    protected:

    private:
        HwWrap_VelocityOutput output;

        float velocityRequested = 0.0;
        float velocityRampRequested = 0.0;

        float velocityCurrentValue = 0.0;

};
