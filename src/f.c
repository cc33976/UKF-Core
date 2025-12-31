#include "f.h"

/* 
F function that returns the f(x) derivative function from EOM. Returns the
velocity v, the acceleratin a, and the jerk j (assumed 0 at small dt)
*/ 

void f(double x[3], double dxdt[3]){
  
  // x is 2D, 3xN matrix holding p, v, a. One row passed @ a time making 1x3.
  dxdt[0] = x[1];
  dxdt[1] = x[2];
  dxdt[2] = 0.0;
  
} // end f
