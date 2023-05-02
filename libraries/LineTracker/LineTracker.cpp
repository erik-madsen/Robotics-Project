/* 
    LineTracker.cpp

    Responsibility:
    Provide the veichles position relative to a line using a row of optical sensors.
    The line tracking is based on the "torque" of a histogram representation of the inputs.
*/

#include "LineTracker.h"
#include "Debug.h"
#include "HwWrap.h"

LineTracker::LineTracker(void)
{
}

void LineTracker::Init(void)
{
    int i;

    sensor[0].pinNo = LOGICAL_INPUT_0;
    sensor[1].pinNo = LOGICAL_INPUT_1;
    sensor[2].pinNo = LOGICAL_INPUT_2;
    sensor[3].pinNo = LOGICAL_INPUT_3;
    sensor[4].pinNo = LOGICAL_INPUT_4;

    sensor[0].sensorBias = 0.09;
    sensor[1].sensorBias = 0.04;
    sensor[2].sensorBias = 0.17;
    sensor[3].sensorBias = 0.11;
    sensor[4].sensorBias = 0.19;

    sensor[0].sensorScaling = 1.00 / 1.00;
    sensor[1].sensorScaling = 1.00 / 1.00;
    sensor[2].sensorScaling = 1.00 / 1.00;
    sensor[3].sensorScaling = 1.00 / 1.00;
    sensor[4].sensorScaling = 1.00 / 1.00;
}

void LineTracker::Update
(
    lineState *lineState,   // Current state of the line tracking
    float *veichlePosition  // Current position of the veichle relative to the line
)
{
    unsigned i;

    // Read sensor inputs, and
    // determine min and max values

    inputMax = 0.0;
    inputMin = 1.0;
    mass   = 0.0;
    torque = 0.0;


    for (i=0; i<NO_OF_SENSORS; i++)
    {
        sensor[i].inputValue = ((float)HwWrap::GetInstance()->AnalogInput(sensor[i].pinNo) / ADC_RANGE);

        sensor[i].inputValue -= sensor[i].sensorBias;
        sensor[i].inputValue *= sensor[i].sensorScaling;
    }

#ifdef SIM_LINE_TRACKER
    SimulateInputs();
#endif

    for (i=0; i<NO_OF_SENSORS; i++)
    {
        if (sensor[i].inputValue > inputMax) inputMax = sensor[i].inputValue;
        if (sensor[i].inputValue < inputMin) inputMin = sensor[i].inputValue;
    }

    // Find the "mass" and "torque" of the histogram

    for (i=0; i<NO_OF_SENSORS; i++)
    {
        mass += (sensor[i].inputValue);
        torque += ((sensor[i].inputValue) * (i+1));
    }

    // Set the position value and state, where the position
    // approximately is the center of the visible part of the line

    if (mass != 0)
    {
        centroid = torque / mass;
    }
    else
    {
        centroid = 0.0;
    }

    if ((inputMax - inputMin) < 0.08)
    {
        // Apparently no line, or line is too wide

        if (stateOfTracking == lineState_LOST_TO_THE_RIGHT ||
            stateOfTracking == lineState_LOST_TO_THE_LEFT  ||
            stateOfTracking == lineState_LOST)
        {
            // Lost state is already recognized
            // Just keep the state and the last position value until the line is found again
        }
        else if (stateOfTracking == lineState_TRACKED_TO_THE_RIGHT)
        {
            // Before it was at the outmost sensor, so now it is probably beyond there
            stateOfTracking = lineState_LOST_TO_THE_RIGHT;
            positionOfLine = 1.0;
        }
        else if (stateOfTracking == lineState_TRACKED_TO_THE_LEFT)
        {
            // Before it was at the outmost sensor, so now it is probably beyond there
            stateOfTracking = lineState_LOST_TO_THE_LEFT;
            positionOfLine = -1.0;
        }
        else
        {
          // Before it was tracked inside the outmost sensors, so now it is probably gone
          stateOfTracking = lineState_LOST;
          positionOfLine = 0.0;
        }
    }
    else if ((sensor[NO_OF_SENSORS-1].inputValue - sensor[NO_OF_SENSORS-2].inputValue) > ((inputMax - inputMin) / 2))
    {
        // Only the rightmost sensor is asserted
        stateOfTracking = lineState_TRACKED_TO_THE_RIGHT;
        positionOfLine = 1.0;
    }
    else if ((sensor[0].inputValue - sensor[1].inputValue) > ((inputMax - inputMin) / 2))
    {
        // Only the leftmost sensor is asserted
        stateOfTracking = lineState_TRACKED_TO_THE_LEFT;
        positionOfLine = -1.0;
    }
    else
    {
        // The line is present and tracked properly
        stateOfTracking = lineState_TRACKED;
        // Set the position in the range [-0.5 - 0.5]
        positionOfLine = (centroid - ((float)(NO_OF_SENSORS+1) / 2)) / ((float)(NO_OF_SENSORS+1) / 2);
    }

    *veichlePosition = -positionOfLine;
    *lineState = stateOfTracking;
}


#ifdef SIM_LINE_TRACKER
void LineTracker::SimulateInputs(void)
{
    // Use a principal representation of the line position like this
    // for debugging purposes:
    // |.....*****.....|
    // Dynamically let the '*' move sidewards and use their positions
    // and use these to simulate the input values.

    // Initialize array to all '.'
    for (int p=0; p<(3*NO_OF_SENSORS); p++)
    {
        simIndicationPoints[p] = '.';
    }

    if (simCountingDir == 0)
    {
        if (simIndicationIndex > 0)
        {
            // Decrement when counting down and not at the limit
            simIndicationIndex--;
        }
        else
        {
            simCountingDir = 1;
        }
    }
    else
    {
        if (simIndicationIndex < (2*NO_OF_SENSORS))
        {
            // Increment when counting up and not at the limit
            simIndicationIndex++;
        }
        else
        {
            simCountingDir = 0;
        }
    }

    // Add the '*' at the indicated position
    for (int s=0; s<NO_OF_SENSORS; s++)
    {
        simIndicationPoints[simIndicationIndex + s] = '*';
    }

    // Simulate the inputs using just an arbitrary value
    for (int p=NO_OF_SENSORS; p<(2*NO_OF_SENSORS); p++)
    {
        if (simIndicationPoints[p] == '*')
        {
            sensor[p - NO_OF_SENSORS].inputValue = 0.2;
        }
        else
        {
            sensor[p - NO_OF_SENSORS].inputValue = 0.0;
        }
    }
}
#endif

void LineTracker::DebugInfo(void)
{
    int i;

    HwWrap::GetInstance()->DebugString(" Max:  ");
    HwWrap::GetInstance()->DebugFloat(inputMax);
    HwWrap::GetInstance()->DebugString("  Torque:  ");
    HwWrap::GetInstance()->DebugFloat(torque);
    HwWrap::GetInstance()->DebugNewLine();

    HwWrap::GetInstance()->DebugString(" Min:  ");
    HwWrap::GetInstance()->DebugFloat(inputMin);
    HwWrap::GetInstance()->DebugString("  Mass:    ");
    HwWrap::GetInstance()->DebugFloat(mass);
    HwWrap::GetInstance()->DebugString("  Centroid:  ");
    HwWrap::GetInstance()->DebugFloat(centroid);
    HwWrap::GetInstance()->DebugNewLine();

    for (i=0; i<NO_OF_SENSORS; i++)
    {
        HwWrap::GetInstance()->DebugString("  ");
        HwWrap::GetInstance()->DebugFloat(sensor[i].inputValue);
    }
    HwWrap::GetInstance()->DebugNewLine();

    for (i=0; i<NO_OF_SENSORS; i++)
    {
        HwWrap::GetInstance()->DebugString("  ");
        HwWrap::GetInstance()->DebugFloat(sensor[i].inputValue - inputMin);
    }
    HwWrap::GetInstance()->DebugNewLine();

    HwWrap::GetInstance()->DebugString(" Pos:  ");
    HwWrap::GetInstance()->DebugFloat(positionOfLine);
    HwWrap::GetInstance()->DebugString("  ");
    switch (stateOfTracking)
    {
        case lineState_UNDEFINED:
            HwWrap::GetInstance()->DebugString("UNDEFINED");
            break;
        case lineState_TRACKED:
            HwWrap::GetInstance()->DebugString(" TRACKED ");
            break;
        case lineState_TRACKED_TO_THE_RIGHT:
            HwWrap::GetInstance()->DebugString("TRACKED_R");
            break;
        case lineState_TRACKED_TO_THE_LEFT:
            HwWrap::GetInstance()->DebugString("TRACKED_L");
            break;
        case lineState_LOST_TO_THE_RIGHT:
            HwWrap::GetInstance()->DebugString(" LOST_R  ");
            break;
        case lineState_LOST_TO_THE_LEFT:
            HwWrap::GetInstance()->DebugString(" LOST_L  ");
            break;
        case lineState_LOST:
            HwWrap::GetInstance()->DebugString("  LOST   ");
            break;
    }
    HwWrap::GetInstance()->DebugString("              |");
    HwWrap::GetInstance()->DebugString( simIndicationPoints );
    HwWrap::GetInstance()->DebugString("|");
    HwWrap::GetInstance()->DebugNewLine();
}
