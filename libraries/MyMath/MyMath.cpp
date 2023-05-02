/*
  A simple math module
*/

#include "Arduino.h"
#include "MyMath.h"

MyMath::MyMath(void)
{
}

unsigned MyMath::sum(unsigned x, unsigned y)
{
  return x+y;
}
