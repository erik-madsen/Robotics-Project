// A class to generate Fibonacci numbers

#include "Fibonacci.h"

Fibonacci::Fibonacci(void)
{
}

unsigned Fibonacci::Get(unsigned position)
{
  unsigned fibo=0;
  unsigned next=1;
  unsigned temp;
  unsigned i;

  for (i=1; i<position; i++)
  {
    temp = fibo + next;
    fibo = next;
    next = temp;
  }

  return fibo;
}
