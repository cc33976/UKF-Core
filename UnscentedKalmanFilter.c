#include <UnscentedKalmanFilter.h>
#include <zeros.h> 
#include <math.h>

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
	      MerweSigmaPoints *sp) {


  // initialize the x matrix as zeros (intial guess)
  for (int i=0; i < dim_x; i++) {
    ukf->x[i] = 0.0;
  } // end outer for loop

  ukf->P = P;
  ukf->Q = Q;
  ukf->R = R;
  ukf->sp = sp;
  ukf->dt = dt;

  // assign function pointers 
  ukf->fx = fx;
  ukf->hx = hx;
  
} // end init_UKF




/*
left off: working on UKF and header (mostly done) and wrote cholesky function.
looking at function declarations in the UKF struct, similarly to python __init__ method.
Working way down the list of functions and declarations needed before moving on from
this file and back to the main.c file. 
 */ 
