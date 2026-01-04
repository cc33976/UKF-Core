#include "residual.h"
#include <stdio.h>

void residual(double a[3], double b[3], double out[3]) {

  for (int i=0; i < 3; i++){
    out[i] = b[i] - a[i];
  } // end for loop

} // end residual 
