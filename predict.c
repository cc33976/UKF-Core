#include "UnscentedKalmanFilter.h"
#include "compute_process_signal.h"
#include "merwe.h"

void predict(UKF *ukf,
	     MerweSigmaPoints *sp,
	     double sigmas[7][3],
	     void (*UT)(MerweSigmaPoints, double[7][3], UnscentedKalmanFilter)){

  // calculate sigma points for given mean & covariance
  compute_process_signal(ukf, sp, sigmas); 

  // pass sigmas through unscented transform to calculate prior
  UT(sp, sigmas, ukf);

  // save prior 
  for (int i = 0; i < 3; i++) {
    ukf->x_prior[i] = ukf->x[i];
    for (int j = 0; j < 3; j++) {
      ukf->P_prior[i][j] = ukf->P[i][j]

    } // end nested for loop
  } // end for loop 
 


} // end predict
