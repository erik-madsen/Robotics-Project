/*

Functional description :
------------------------
The file contains the definition of a class used for 
PID regulation.

Until the PID regulator is enabled, the output is 0.0
The default range of the output from the PID regulator 
is [0.0 .. 1.0] but it can be changed to [-1.0 .. 1.0]

Trimming the PID regulator :
----------------------------
A simple method to achieve an initial, usable setting of 
the PID regulator parameters is as follows:
 - Set Kp, Ki, and Kd to zero.
 - Gradually increase the Kp until the output of the PID regulator
   becomes unstable.
   Then use a value of Kp that is approximately 50% of this.
 - Gradually increase the Ki until the output of the PID regulator
   becomes unstable.
   Then use a value of Ki that is approximately 80% of this.
 - Gradually increase the Kd until the output of the PID regulator
   becomes unstable.
   Then use a value of Kd that is approximately 80% of this.
The PID regulator may afterwards need a mores specific adjustment 
with respect to the actual regulation loop and it purpose.

Special notes :
---------------
The limit on the summarized error is set so that the i_term
will not exceed the maximum output of the regulator

*/

#include "PIDregulator.h"
#include "Common.h"

t_boolean enabled;
t_boolean range_to_minus_one;

float Kp;
float Ki;
float Kd;

float error_sum;
float error_sum_max;
float error_last;

float p_term;
float i_term;
float d_term;

void PID_constructor(void)
{
  enabled = FALSE;
  range_to_minus_one = FALSE;

  Kp = 0.10f;
  Ki = 0.05f;
  Kd = 0.00f;

  error_sum = 0.0f;
  error_sum_max = 1.0f / Ki;
  error_last = 0.0f;
}

void PID_ResetInternalValues(void)
{
  error_sum = 0.0f;
  error_sum_max = 1.0f / Ki;
  error_last = 0.0f;
}

void PID_Init(void)
{
  enabled = TRUE;
}

void PID_SetRangeToIncludeMinusOne(t_boolean setting)
{
  range_to_minus_one = setting;
}

void PID_SetKp(float value)
{
  Kp = value;
}

void PID_SetKi(float value)
{
  Ki = value;

  if (Ki != 0.0)
  {
    error_sum_max = 1.0 / Ki;
  }
}

void PID_SetKd(float value)
{
  Kd = value;
}

void PID_GetParams(t_PID_parameters* param_ptr)
{
  // OS_EnterRegion();
  // ... just to make sure that the parameters are coherent

  param_ptr->Kp = Kp;
  param_ptr->Ki = Ki;
  param_ptr->Kd = Kd;

  param_ptr->p_term = p_term;
  param_ptr->i_term = i_term;
  param_ptr->d_term = d_term;

  param_ptr->error_sum = error_sum;

  // OS_LeaveRegion();
}

float PID_Update(
/****************************************************************************

Update the PID regulator based on the current error.

Until the PID regulator is enabled, the output is 0.0
The default range of the output from the PID regulator 
is [0.0 .. 1.0] but it can be changed to [-1.0 .. 1.0]

****************************************************************************/
float error  // the actual error of the regulated parameter in the range [-1.0 .. 1.0]
)
{
  float output;

  if (!enabled)
  {
    error = 0.0;
  }

  if (error >  1.0) error =  1.0;
  if (error < -1.0) error = -1.0;

  error_sum += error;

  if (error_sum >  error_sum_max) error_sum =  error_sum_max;
  if (range_to_minus_one)
  {
    if (error_sum < -error_sum_max) error_sum = -error_sum_max;
  } 
  else
  {
    if (error_sum <  0.0)           error_sum =  0.0;
  }

  p_term = Kp * error;
  i_term = Ki * error_sum;
  d_term = Kd * (error - error_last);

  error_last = error;

  output = p_term + i_term + d_term;

  if (output >  1.0) output =  1.0;
  if (range_to_minus_one)
  {
    if (output < -1.0) output = -1.0;
  }
  else
  {
    if (output <  0.0) output =  0.0;
  }

  return output;
}
