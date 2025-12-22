#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "cross_variance.h"

void update(UKF *ukf,
	    MerweSigmaPoints *sp,
	    double R[3][3],
	    void (*UT)(MerweSigmaPoints, double[7][3], UnscentedKalmanFilter),
	    double H[3][3],
	    double z[3]) {

  // pass prior sigmas (sigmas_f) through h(x) to get measurement sigmas
  for (int i = 0; i < 7; i++) {
    h_func(ukf->sigmas_f[i], H, ukf->sigmas_h[i]);
  } // end for loop

  // pass mean and covariance through unscented transform
  double zp[3];
  UT(ukf->sigmas_h, ukf->Wm, ukf->Wc, R, ukf, zp, ukf->S);
  
  // compute the cross variance of state and measurements
  double Pxz[3][3];
  cross_variance(ukf, z, Pxz);
  
  
  /* compute the Kalman gain:
     
     K = Pxz * S^-1

     Factor: S = L * L^T

     Solve: L * y = Pxz^T

     Solve: L^T * x = y

     Then: K = x^T (not the average state)

  */
  
  double L_SI[3][3];
  cholesky(ukf->S,L_SI);
  compute_K_cholesky(Pxz,L_SI,ukf->K);
  
  // compute residual between measurement and prediction
  
  
  // update Gaussian state estimate (x,P)
  
  
  // save measurement and posterior state 
  

  

} // end update
