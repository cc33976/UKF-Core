#include "subtract.h"
#include <stdio.h>

/*
  The function is called subtract to remain synonymous with
  the python UKF function names, but should be called
  'distribute' as it adds three sigmas above and below the
  average value.
*/ 

void subtract(double x[3], double U[3][3], double sigmas[7][3]){

  /*
  printf("Entering the subtract function\n");
  printf("printing inputs:\n");
  printf("x:\n");
  printf("[");
  for (int i=0;i<3;i++){
    printf("%.3f ",x[i]);
  }
  printf("]\n");

  printf("U:\n");
  for (int i=0;i<3;i++){
    printf("[");
    for (int j=0;j<3;j++){
      printf("%.3f ",U[i][j]);
    }
    printf("]\n");
  }
  */
  

  for (int i = 0; i < 3; i++){
    for (int j=0; j < 3; j++){
      // transpose of U because is lower triangular
      sigmas[i+1][j] = x[j] - U[j][i];
      //printf("lower sigma (-): %.4f\n",sigmas[i+1][j]);
      sigmas[i+4][j] = x[j] + U[j][i];
      //printf("upper sigma (+): %.4f\n",sigmas[i+4][j]);
    } // end nested for loop
  } // end outer for loop


} // end subtract 

