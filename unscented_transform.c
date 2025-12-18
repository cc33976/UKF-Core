#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "residual.h"


void unscented_transform(MerweSigmaPoints *sp,
			 double sigmas[7][3],
			 UKF *ukf){

  // compute new average by taking weighted sum of sigma points
  dot(sp->Wm, sigmas, ukf->x);

  // compute new covariance matrix
for (int k = 0; k < 7; k++) {
        residual_fn(sigmas[k], ukf->x, y);  // y = sigmas[k] - x

        // P += Wc[k] * outer(y, y)
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                ukf->P[i][j] += Wc[k] * y[i] * y[j];
		
            } // end inner for loop 
        } // end middle for loop
 } // end k for loop
  

} // end unscented_transform
