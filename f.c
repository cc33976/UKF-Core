#include <stdio.h>
#include "allocate.h"

/*
F function that returns the f(x) derivative function from EOM. Returns the
velocity v, the acceleratin a, and the jerk j (assumed 0 at small dt)
*/ 

void f(double x[3], double Fx[3]){
  // x is 2D, 3xN matrix holding p, v, a. One row passed @ a time making 1x3.
  double v = x[1];
  double a = x[2];
  double j = 0;

  // allocate new array F(x) for RK4 
  double** Fx[3]

  Fx[0] = v;
  Fx[1] = a;
  Fx[2] = j;

  // returns 1x3 array F(x)
  
}
