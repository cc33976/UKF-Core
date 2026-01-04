#include "cross_variance.h"
#include "UnscentedKalmanFilter.h"
#include "residual.h"
#include <stdio.h>

void cross_variance(UKF *ukf,
		    MerweSigmaPoints *sp,
		    double zp[3],
		    double Pxz[3][3]){


  // compute cross variance of the state x and measurement z
  double dx[3];
  double dz[3];

  // set cross variance to zero
  for (int r=0; r<3; r++){
    for (int c=0; c<3; c++){
      Pxz[r][c] = 0.0;
	} // end nested for loop (c)
  } // end outer for loop (r)


  // calculate residuals and add to cross variance
  for (int i = 0; i < 7; i++){
    residual(ukf->sigmas_f[i], ukf->x_prior, dx); // was ukf->x
    residual(ukf->sigmas_h[i], zp, dz);

    for (int r=0; r<3; r++){
      for (int c=0; c<3; c++){
	Pxz[r][c] += sp->Wc[i] * dx[r] * dz[c];
      } // end innermost for loop (c)
    } // end middle for loop (r)
  } // end for loop

  
} // end cross_variance
