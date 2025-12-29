#include "UnscentedKalmanFilter.h"


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


  // initialize the x matrix as zeros (intial guess)
  for (int i=0; i < dim_x; i++) {
    ukf->x[i] = 0.0;
  } // end outer for loop

  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      ukf->P[i][j] = P[i][j];
      ukf->Q[i][j] = Q[i][j];
      ukf->R[i][j] = R[i][j];
    }
  }
  ukf->sp = sp;
  ukf->dt = dt;

  // assign function pointers 
  ukf->fx = fx;
  ukf->hx = hx;
  
} // end init_UKF


