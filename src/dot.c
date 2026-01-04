#include "dot.h"

// might be a good idea to make Wm and x_new ambiguous (a,b) 
void dot(double A[7], double B[7][3], double Out[3]) {

  double sum = 0.0;
  
  for (int j=0; j < 3; j++){
    sum = 0.0;
    for (int i=0; i< 7; i++){
      sum = sum + (A[i] * B[i][j]);
    } // end nested for loop
    Out[j] = sum;
  } // end for loop

} // end dot 
