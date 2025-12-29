#include "update.h"
#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "cross_variance.h"
#include "residual.h"
#include "trans33.h"
#include "dot33.h"
#include "dot3.h"
#include "h_func.h"
#include "compute_K_cholesky.h"

void update(UKF *ukf,
	    MerweSigmaPoints *sp,
	    double R[3][3],
	    void (*UT)(double sigmas[7][3],
		       double Wm[7],
		       double Wc[7],
		       double noise_cov[3][3],
		       UKF *ukf,
		       double x[3],
		       double P[3][3]),
	    double H[3][3],
	    double z[3]) {

  // pass prior sigmas (sigmas_f) through h(x) to get measurement sigmas
  for (int i = 0; i < 7; i++) {
    h_func(ukf->sigmas_f[i], H, ukf->sigmas_h[i]);
  } // end for loop

  // pass mean and covariance through unscented transform
  double zp[3];
  UT(ukf->sigmas_h, sp->Wm, sp->Wc, R, ukf, zp, ukf->S);
  
  // compute the cross variance of state and measurements
  double Pxz[3][3];
  cross_variance(ukf, sp, z, Pxz);
  
  
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
  residual(z,zp,ukf->y);
  
  // update Gaussian state estimate (x,P)
  double temp_x[3];
  dot3(ukf->y, ukf->K, temp_x);

  double Kt[3][3];
  trans33(ukf->K,Kt);
  double dot_S_Kt[3][3];
  dot33(ukf->S, Kt, dot_S_Kt);
  double dot_K_dot_S_Kt[3][3];
  dot33(ukf->K, dot_S_Kt, dot_K_dot_S_Kt);

  for (int i = 0; i < 3; i++){
    ukf->x[i] += temp_x[i];
    for (int j = 0; j < 3; j++){
      ukf->P[i][j] -= dot_K_dot_S_Kt[i][j];
    } // end nested for loop
  } // end for loop
  
  // save measurement and posterior state 
  for (int i=0; i < 3; i++){
    ukf->x_post[i] = ukf->x[i];
    for (int j=0; j<3; j++){
      ukf->P_post[i][j] = ukf->P[i][j];
    } // end nested for loop
  } // end outer for loop

  

} // end update
