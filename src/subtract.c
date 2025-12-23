#include "subtract.h" 

/*
  The function is called subtract to remain synonymous with
  the python UKF function names, but should be called
  'distribute' as it adds three sigmas above and below the
  average value.
*/ 

void subtract(double x[3], double U[3][3], double sigmas[7][3]){

  for (int i = 0; i < 3; i++){
    for (int j=0; j < 3; j++){
      sigmas[i+1][j] = x[j] - U[i][j];
      sigmas[i+4][j] = x[j] + U[i][j]; 
    } // end nested for loop
  } // end outer for loop


} // end subtract 

