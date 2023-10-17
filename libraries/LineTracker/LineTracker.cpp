/*
    LineTracker.cpp

    Responsibility:
    ---------------
    Provide the vehicles position relative to a line using a row of optical sensors.
    The line tracking is based on the "torque" of a histogram representation of the inputs.

    An instances of this class is an "active object" and
    it's "Update" function must be called on a regular basis.
*/

#include "LineTracker.h"
#include "Debug.h"
#include "HwWrap.h"
#include "math.h"

LineTracker::LineTracker
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
}

void LineTracker::Init
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    int i;

    sensor[0].pinNo = ANALOG_INPUT_0;
    sensor[1].pinNo = ANALOG_INPUT_1;
    sensor[2].pinNo = ANALOG_INPUT_2;
    sensor[3].pinNo = ANALOG_INPUT_3;
    sensor[4].pinNo = ANALOG_INPUT_4;

    sensor[0].sensorBias = 0.00;
    sensor[1].sensorBias = 0.00;
    sensor[2].sensorBias = 0.00;
    sensor[3].sensorBias = 0.00;
    sensor[4].sensorBias = 0.00;

    sensor[0].sensorScaling = 1.00;
    sensor[1].sensorScaling = 1.00;
    sensor[2].sensorScaling = 1.00;
    sensor[3].sensorScaling = 1.00;
    sensor[4].sensorScaling = 1.00;
}

void LineTracker::Update
//  --------------------------------------------------------------------------------
(
    lineState *lineState,   // Current state of the line tracking
    float *vehiclePosition  // Current position of the vehicle relative to the line
)
//  --------------------------------------------------------------------------------
{
    unsigned i;

#define CENTROID_VALUE_OF_ONE_SENSOR_POSITION   (0.5)

#define CENTROID_VALUE_OF_LEFTMOST_SENSOR       (1.0)
#define CENTROID_VALUE_OF_RIGHTMOST_SENSOR      ((float)(NO_OF_SENSORS))

#define CENTROID_VALUE_OUTSIDE_LEFT             (CENTROID_VALUE_OF_LEFTMOST_SENSOR - CENTROID_VALUE_OF_ONE_SENSOR_POSITION)
#define CENTROID_VALUE_OUTSIDE_RIGHT            (CENTROID_VALUE_OF_RIGHTMOST_SENSOR + CENTROID_VALUE_OF_ONE_SENSOR_POSITION)

#define CENTROID_VALUE_CENTER                   ((float)(NO_OF_SENSORS+1) / 2)

    // Read sensor inputs, and
    // determine min and max values

    mass   = 0.0;
    torque = 0.0;

    for (i=0; i<NO_OF_SENSORS; i++)
    {
        sensor[i].inputValue = ((float)HwWrap::GetInstance()->AnalogInput(sensor[i].pinNo) / ADC_RANGE);

        sensor[i].inputValue -= sensor[i].sensorBias;
        sensor[i].inputValue *= sensor[i].sensorScaling;
    }

#ifdef LINE_TRACKER_USE_SIMULATION
    SimulateInputs();
#endif

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
        centroid = CENTROID_VALUE_CENTER;
    }

    if ( fabs( centroid - CENTROID_VALUE_CENTER ) < (CENTROID_VALUE_OF_ONE_SENSOR_POSITION / 2) )
    {
        // The centroid is closer than half a "sensor position" from the center, so apparently no line, or line is too wide

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
            centroid = CENTROID_VALUE_OUTSIDE_RIGHT; // Mainly for debugging
            positionOfLine = 1.0;
        }
        else if (stateOfTracking == lineState_TRACKED_TO_THE_LEFT)
        {
            // Before it was at the outmost sensor, so now it is probably beyond there
            stateOfTracking = lineState_LOST_TO_THE_LEFT;
            centroid = CENTROID_VALUE_OUTSIDE_LEFT; // Mainly for debugging
            positionOfLine = -1.0;
        }
        else
        {
          // Before it was tracked inside the outmost sensors, so now it is probably gone
          stateOfTracking = lineState_LOST;
          positionOfLine = 0.0;
        }
    }
    else if (centroid >= CENTROID_VALUE_OF_RIGHTMOST_SENSOR - (CENTROID_VALUE_OF_ONE_SENSOR_POSITION / 2))
    {
        // Only the rightmost sensor is asserted
        stateOfTracking = lineState_TRACKED_TO_THE_RIGHT;
        positionOfLine = (centroid - CENTROID_VALUE_CENTER) / CENTROID_VALUE_CENTER;
    }
    else if (centroid <= CENTROID_VALUE_OF_LEFTMOST_SENSOR + (CENTROID_VALUE_OF_ONE_SENSOR_POSITION / 2))
    {
        // Only the leftmost sensor is asserted
        stateOfTracking = lineState_TRACKED_TO_THE_LEFT;
        positionOfLine = (centroid - CENTROID_VALUE_CENTER) / CENTROID_VALUE_CENTER;
    }
    else
    {
        // The line is present and tracked properly
        stateOfTracking = lineState_TRACKED;
        positionOfLine = (centroid - CENTROID_VALUE_CENTER) / CENTROID_VALUE_CENTER;
    }

    *vehiclePosition = -positionOfLine;
    *lineState = stateOfTracking;
}


#ifdef LINE_TRACKER_USE_SIMULATION
void LineTracker::SimulateInputs
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    // For debugging purposes, use a principal representation of the line position like this,
    // indicating the line and the line tracker:
    // 
    //   pos = 0   pos = 2*NO_OF_SENSORS
    //   v         v
    //   .....=====.....    Moving line to track
    //
    //     || ***** ||      Line tracker on the vehicle
    // 
    // Dynamically let the '=' move sidewards and
    // use their positions to simulate the input values.

    // Initialize array
    for (int s=0; s<(3*NO_OF_SENSORS); s++)
    {
        simIndicationPoints[s] = 0;
    }

    // Simulate inputs of the imaginary line at the indicated position
    for (int a=0; a<NO_OF_SENSORS; a++)
    {
        simIndicationPoints[simIndicationIndex + a] = 1;
    }

    // Simulate the inputs using just an arbitrary value
    for (int p=NO_OF_SENSORS; p<(2*NO_OF_SENSORS); p++)
    {
        if (simIndicationPoints[p] == 1)
        {
            sensor[p - NO_OF_SENSORS].inputValue = 0.2;
        }
        else
        {
            sensor[p - NO_OF_SENSORS].inputValue = 0.0;
        }
    }

    // Update the simulated position
    if (simCountingDir == 0)
    {
        if (simIndicationIndex > 0)
        {
            // Decrement when counting down and not at the limit
            simIndicationIndex--;
        }
        else
        {
            // Increment and change counting direction
            simIndicationIndex++;
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
            // Decrement and change counting direction
            simIndicationIndex--;
            simCountingDir = 0;
        }
    }
}
#endif

void LineTracker::DebugInfo
//  --------------------------------------------------------------------------------
(
    void
)
//  --------------------------------------------------------------------------------
{
    int i;
    char lineIndication[(3*NO_OF_SENSORS + 1)];

    // Initialize array for line indication
    for (int s=0; s<(3*NO_OF_SENSORS); s++)
    {
        lineIndication[s] = '.';
    }
    // Mark the imaginary line at the indicated position
    for (int a=0; a<NO_OF_SENSORS; a++)
    {
        lineIndication[unsigned(centroid*2)-1 + a] = '=';
    }
    lineIndication[3*NO_OF_SENSORS] = 0;

    HwWrap::GetInstance()->DebugNewLine();

#ifdef LINE_TRACKER_USE_DEBUGGING_VERBOSE
    HwWrap::GetInstance()->DebugString(" inputValue:");
    for (i=0; i<NO_OF_SENSORS; i++)
    {
        HwWrap::GetInstance()->DebugString("  ");
        HwWrap::GetInstance()->DebugFloat(sensor[i].inputValue);
    }
    HwWrap::GetInstance()->DebugNewLine();
#endif

    HwWrap::GetInstance()->DebugString(" Torque:  ");
    HwWrap::GetInstance()->DebugFloat(torque);
    HwWrap::GetInstance()->DebugString("   Centroid:  ");
    HwWrap::GetInstance()->DebugFloat(centroid);
    HwWrap::GetInstance()->DebugString("    ");
    HwWrap::GetInstance()->DebugString( lineIndication );

    HwWrap::GetInstance()->DebugString("  ");
    switch (stateOfTracking)
    {
        case lineState_UNDEFINED:
            HwWrap::GetInstance()->DebugString("UNDEFINED");
            break;
        case lineState_TRACKED:
            HwWrap::GetInstance()->DebugString("TRACKED");
            break;
        case lineState_TRACKED_TO_THE_RIGHT:
            HwWrap::GetInstance()->DebugString("TRACKED_R");
            break;
        case lineState_TRACKED_TO_THE_LEFT:
            HwWrap::GetInstance()->DebugString("TRACKED_L");
            break;
        case lineState_LOST_TO_THE_RIGHT:
            HwWrap::GetInstance()->DebugString("LOST_R");
            break;
        case lineState_LOST_TO_THE_LEFT:
            HwWrap::GetInstance()->DebugString("LOST_L");
            break;
        case lineState_LOST:
            HwWrap::GetInstance()->DebugString("LOST");
            break;
    }
    HwWrap::GetInstance()->DebugNewLine();

    HwWrap::GetInstance()->DebugString(" Mass:    ");
    HwWrap::GetInstance()->DebugFloat(mass);
    HwWrap::GetInstance()->DebugString("   Line pos: ");
    if (positionOfLine >= 0.0)
        HwWrap::GetInstance()->DebugString(" ");
    HwWrap::GetInstance()->DebugFloat(positionOfLine);
    HwWrap::GetInstance()->DebugString("         ^^^^^");
    HwWrap::GetInstance()->DebugNewLine();

    HwWrap::GetInstance()->DebugNewLine();
}
