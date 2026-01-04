#include "dot3.h"

// might be a good idea to make Wm and x_new ambiguous (a,b) 
void dot3(double m[3][3], double v[3], double out[3]) {
  
  for (int i=0; i < 3; i++){
    double sum = 0.0;
    for (int j=0; j< 3; j++){
      sum += (m[i][j] * v[j]);
    } // end nested for loop
    out[i] = sum;
  } // end for loop

} // end dot 
