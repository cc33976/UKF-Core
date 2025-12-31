#include <stdio.h>
#include <math.h>
#include "cholesky.h"

void cholesky(const double A[3][3], double L[3][3]) {

    if (A[0][0] <= 0.0) {
        printf("Cholesky error: A[0][0]=%f <= 0\n", A[0][0]);
        return;
    } // end if

    /*
    printf("A:\n");
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 3; j++){
	printf("%.3f ",A[i][j]);
      }
      printf("\n");
    }
    */

    // initialize L to zeros
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            L[i][j] = 0.0;
	    // printf("%.3f ",L[i][j]);
        } // end nested for loop
	//printf("\n");
    } // end outer for loop
    

    // column by column
    L[0][0] = sqrt(A[0][0]);
    L[1][0] = A[1][0] / L[0][0];
    L[2][0] = A[2][0] / L[0][0];

    double temp = A[1][1] - L[1][0]*L[1][0];

    // prevent tiny negatives from sneaking in
    double eps = 1e-12;
    if (temp < eps){
      printf("Cholesky warning: temp to small, switching to eps\n");
      temp = eps;
    } // end if
    
    if (temp <= 0) {
        printf("Cholesky error: A[1][1]-L[1][0]^2=%f <= 0\n", temp);
        return;
    } // end if 
    L[1][1] = sqrt(temp);
    L[2][1] = (A[2][1] - L[2][0]*L[1][0]) / L[1][1];

    temp = A[2][2] - L[2][0]*L[2][0] - L[2][1]*L[2][1];
    if (temp <= 0) {
        printf("Cholesky error: A[2][2]-sum=%f <= 0\n", temp);
        return;
    } // end if 
    L[2][2] = sqrt(temp);

    /*
    // print L matrix
    printf("L:\n");
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 3; j++){
	printf("%.3f ",L[i][j]);
      }
      printf("\n");
    }
    */
} // end cholesky

