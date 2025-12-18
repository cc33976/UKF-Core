#include "UnscentedKalmanFilter.h"
#include "compute_process_signal.h"

void predict(UKF *ukf,
	     double dt,
	     void (*UT)(MerweSigmaPoints, double[7][3], double[3], double[3][3]),
	     void (*fx)(double[3], double, double[3])){

  // calculate sigma points for given mean & covariance
  compute_process_signal(); 

  // pass sigmas through unscented transform to calculate prior
  UT();

  // save prior 
  
 


} // end predict
