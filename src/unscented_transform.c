#include "unscented_transform.h"
#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "residual.h"
#include "unscented_transform.h"
#include "dot.h"
#include <stdio.h>

// returns state and covariance matrices (x,P)
void unscented_transform(double sigmas[7][3],
			 double Wm[7],
			 double Wc[7],
			 double noise_cov[3][3],
			 UKF *ukf,
			 double x[3],
			 double P[3][3]){


  // set P to zero before accumulating
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      P[i][j] = 0.0;
    } // end for
  } // end for 

  // compute new average state by taking weighted sum of sigma points
  dot(Wm, sigmas, x);

  // compute new covariance matrix
  double y[3] = {0};
  for (int k = 0; k < 7; k++) {
    residual(sigmas[k], x, y);  // y = sigmas[k] - x
    
    // P += Wc[k] * outer(y, y)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
	P[i][j] += Wc[k] * y[i] * y[j];
		
      } // end inner for loop 
    } // end middle for loop
  } // end k for loop

 
  // use R as noise_cov 
  if (!(noise_cov == NULL)) {
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 3; j++){
	P[i][j] += noise_cov[i][j];
      } // end nested for loop
    } // end for loop
  } // end if statement

 

} // end unscented_transform
