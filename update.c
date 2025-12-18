#include "UnscentedKalmanFilter.h"
#include "merwe.h"

void update(UKF *ukf,
	    MerweSigmaPoints *sp,
	    double R[3][3],
	    void (*UT)(MerweSigmaPoints, double[7][3], UnscentedKalmanFilter),
	    double H[3][3]) {

  // pass prior sigmas (sigmas_f) through h(x) to get measurement sigmas
  for (int i = 0; i < 7; i++) {
    h_func(ukf->sigmas_f[i], H, ukf->sigmas_h[i]);
  } // end for loop

  // pass mean and covariance through unscented transform

  // compute the cross variance of state and measurements

  // compute the Kalman gain

  // compute residual between measurement and prediction

  // update Gaussian state estimate (x,P)

  // save measurement and posterior state 
  

  

} // end update
