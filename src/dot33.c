#include "dot33.h"

// computes the dot product of two 3x3 matrices
void dot33(double A[3][3], double B[3][3], double Out[3][3]) {

  double sum = 0.0; 
  
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      sum += A[i][j] * B[j][i];
	for (int k = 0; k < 3; k++){
	  sum += A[i][k] * B[k][j]; 
	} // end innermost for loop
      Out[i][j] = sum;
    } // end nested for loop
  } // end outer for loop

} // end dot33 
