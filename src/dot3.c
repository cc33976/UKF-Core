#include "dot3.h"

// might be a good idea to make Wm and x_new ambiguous (a,b) 
void dot3(double a[3], double b[3][3], double out[3]) {

  double sum = 0.0;
  
  for (int j=0; j < 3; j++){
    for (int i=0; i< 3; i++){
      sum = sum + (a[i] * b[i][j]);
    } // end nested for loop
    out[j] = sum;
    sum = 0.0;
  } // end for loop

} // end dot 
