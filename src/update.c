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
#include "unscented_transform.h"
#include <stdio.h>

void update(UKF *ukf,
	    MerweSigmaPoints *sp,
	    double R[3][3],
	    double H[3][3],
	    double z[3]) {


  // pass prior sigmas (sigmas_f) through h(x) to get measurement sigmas
  for (int i = 0; i < 7; i++) {
    h_func(ukf->sigmas_f[i], H, ukf->sigmas_h[i]);
  } // end for loop

  
  // pass mean and covariance through unscented transform
  double zp[3];
  unscented_transform(ukf->sigmas_h, sp->Wm, sp->Wc, R, ukf, zp, ukf->S);

  
  // compute the cross variance of state and measurements
  double Pxz[3][3];
  cross_variance(ukf, sp, zp, Pxz); // was z not zp

  
  
  /* compute the Kalman gain:
     
     K = Pxz * S^-1

     Factor: S = L * L^T

     Solve: L * y = Pxz^T

     Solve: L^T * x = y

     Then: K = x^T (not the average state)

  */
  
  double L_SI[3][3];
  cholesky(ukf->S,L_SI);

  double K_temp[3][3] = {0};
  compute_K_cholesky(Pxz,L_SI,K_temp); // computes transposed K
  trans33(K_temp, ukf->K); // transpose back into K and store   
  
  // compute residual between measurement and prediction
  residual(zp,z,ukf->y);

  
  //=========== update Gaussian state estimate (x,P)==========
  
  // update x 
  double temp_x[3]; 
  dot3(ukf->K, ukf->y, temp_x); // was y,K =============###########
  

  // Joseph's stabilized covariance update P
  double temp1[3][3]; // I - KH
  double temp1T[3][3]; // temp1^T
  double temp2[3][3]; // temp1 * P_prior
  double temp3[3][3]; // temp2 * temp1^T
  double temp4[3][3]; // K * R
  double temp5[3][3]; // temp4 * K^T
  double KT[3][3]; // K^T
  double I[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};

  // compute temp1
  for (int i=0; i<3; i++){
    for (int j=0; j<3; j++){
      temp1[i][j] = I[i][j] - ukf->K[i][j]; // H is eye(1) 
    }
  }

  dot33(temp1, ukf->P_prior, temp2);

  // compute transpose of temp1
  trans33(temp1,temp1T);

  dot33(temp2, temp1T, temp3);

  dot33(ukf->K, R, temp4);

  // compute transpose of K -- K^T
  trans33(ukf->K,KT);
  dot33(temp4, KT, temp5);


  // ============================================================

  for (int i = 0; i < 3; i++){
    ukf->x[i] += temp_x[i];
    for (int j = 0; j < 3; j++){
      ukf->P[i][j] = temp3[i][j] + temp5[i][j];
    } // end nested for loop
  } // end for loop
  

  for (int i = 0; i < 3; i++){
    ukf->x_post[i] = ukf->x[i];
    for (int j = 0; j < 3; j++){
      ukf->P_post[i][j] = ukf->P[i][j];
    } // end nested for loop
  } // end for loop



} // end update
