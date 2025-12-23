#include "predict.h" 
#include "UnscentedKalmanFilter.h"
#include "compute_process_sigmas.h"
#include "merwe.h"

void predict(UKF *ukf,
	     MerweSigmaPoints *sp,
	     double sigmas[7][3],
	     void (*UT)(double[7][3], double[7], double[7],
			double[3][3], UKF *, double[3] double[3][3])
	     ){

  // calculate sigma points for given mean & covariance
  compute_process_signal(ukf, sp, sigmas); 

  // pass sigmas through unscented transform to calculate prior
  UT(sigmas, ukf->Wm, ukf->Wc, Null, ukf, ukf->x, ukf->P); 

  // save prior 
  for (int i = 0; i < 3; i++) {
    ukf->x_prior[i] = ukf->x[i];
    for (int j = 0; j < 3; j++) {
      ukf->P_prior[i][j] = ukf->P[i][j]

    } // end nested for loop
  } // end for loop 
 


} // end predict
