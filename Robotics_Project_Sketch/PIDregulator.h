/*
  The file contains the definition of a PID regulator.
*/

#include "Common.h"

#ifndef __PIDREGULATOR_H
#define __PIDREGULATOR_H

// Include section

// Define section
typedef struct
{
  float Kp;
  float Ki;
  float Kd;

  float p_term;
  float i_term;
  float d_term;

  float error_sum;
}
t_PID_parameters;

void  PID_constructor(void);

void  PID_ResetInternalValues(void);
void  PID_Init(void);
float PID_Update(float error);                  // Update the PID regulator based on the current error

void  PID_SetRangeToIncludeMinusOne(t_boolean setting);
void  PID_SetKp(float value);
void  PID_SetKi(float value);
void  PID_SetKd(float value);
void  PID_GetParams(t_PID_parameters* param_ptr);

#endif
