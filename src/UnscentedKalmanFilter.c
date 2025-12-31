#include "UnscentedKalmanFilter.h"
#include <stdio.h>
#include <stdlib.h>


// create UKF constructor-like struct

void init_UKF(UKF *ukf,
	      int dim_x,
	      int dim_z,
	      double P[3][3],
	      double Q[3][3],
	      double R[3][3],
	      double dt,
	      fx_fn fx,
	      hx_fn hx,
	      struct MerweSigmaPoints *sp) {

  printf("Entered the init_UKF function\n");

  ukf->x[0] = 0.2;
  ukf->x[1] = 0.1;
  ukf->x[2] = 9.8;

  for (int i=0; i<3; i++){
    ukf->x_post[i] = ukf->x[i];
    for (int j=0; j < 3; j++){
      ukf->P_post[i][j] = ukf->P[i][j];
    }
  }

  //printf("allocated x's as zeros for initial guess\n");

  
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      //printf("attempting P allocation at index (%d,%d)\n",i,j);
      ukf->P[i][j] = P[i][j];
      //printf("attempting Q allocation at index (%d,%d)\n",i,j);
      ukf->Q[i][j] = Q[i][j];
      //printf("attempting R allocation at index (%d,%d)\n",i,j); 
      ukf->R[i][j] = R[i][j];
    } // end nested for loop
  } // end outer for loop

  //printf("Successfully allocated P,Q,R matrices to UKF 'object'\n");

  //printf("Attempting to assign sp and dt values in UKF 'object'\n");
  ukf->sp = sp;
  ukf->dt = dt;
  //printf("Assigned sp and dt values\n");

  //printf("Attempting to assign fx and hx function pointers in UKF 'object'\n");
  // assign function pointers 
  ukf->fx = fx;
  ukf->hx = hx;
  //printf("Assigned fx and hx function pointers\n");

  
  
} // end init_UKF


