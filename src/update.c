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

  printf("###### entering update: ######\n");

    printf("sigmas_f:\n");
	 for (int i = 0; i < 7; i++){
	   printf("[");
	   for (int j = 0; j < 3; j++){
	     printf("%.4f ",ukf->sigmas_f[i][j]);
	   }
	   printf("]\n");
	 }


  // pass prior sigmas (sigmas_f) through h(x) to get measurement sigmas
  for (int i = 0; i < 7; i++) {
    h_func(ukf->sigmas_f[i], H, ukf->sigmas_h[i]);
  } // end for loop

  printf("sigmas_h:\n");
	 for (int i = 0; i < 7; i++){
	   printf("[");
	   for (int j = 0; j < 3; j++){
	     printf("%.4f ",ukf->sigmas_h[i][j]);
	   }
	   printf("]\n");
	 }
  
  // pass mean and covariance through unscented transform
  double zp[3];
  unscented_transform(ukf->sigmas_h, sp->Wm, sp->Wc, R, ukf, zp, ukf->S);

  printf("zp: ");
  for (int i = 0; i < 3; i++){
    printf("%.4f ",zp[i]);
  }
  printf("\n");

 
  printf("S:\n");
  for (int i = 0; i < 3; i++){
    printf("[");
    for (int j = 0; j < 3; j++){
      printf("%.3f ",ukf->S[i][j]);
    }
    printf("]\n");
  }


  
  // compute the cross variance of state and measurements
  double Pxz[3][3];
  cross_variance(ukf, sp, zp, Pxz); // was z not zp

  printf("Pxz:\n");
  for (int i =0; i < 3; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",Pxz[i][j]);
    }
    printf("]\n");
  }
  
  
  /* compute the Kalman gain:
     
     K = Pxz * S^-1

     Factor: S = L * L^T

     Solve: L * y = Pxz^T

     Solve: L^T * x = y

     Then: K = x^T (not the average state)

  */
  
  double L_SI[3][3];
  cholesky(ukf->S,L_SI);

  printf("L_SI:\n");
  for (int i = 0; i < 3; i++){
    printf("[");
    for (int j = 0; j < 3; j++){
      printf("%.4f ",L_SI[i][j]);
    }
    printf("]\n");
  }
  
  compute_K_cholesky(Pxz,L_SI,ukf->K);

  printf("K:\n");
  for (int i = 0; i < 3; i++){
    printf("[");
    for (int j = 0; j < 3; j++){
      printf("%.4f ",ukf->K[i][j]);
    }
    printf("]\n");
  }
  
  
  // compute residual between measurement and prediction
  residual(z,zp,ukf->y);

  printf("y:\n");
  printf("[");
  for (int i = 0; i < 3; i++){
    printf("%.4f ",ukf->y[i]);
  }
  printf("]\n");
  
  //=========== update Gaussian state estimate (x,P)==========
  
  // update x 
  double temp_x[3];
  dot3(ukf->y, ukf->K, temp_x);
  
  printf("temp_x:\n");
  printf("[");
  for (int i = 0; i < 3; i++){
    printf("%.4f ",temp_x[i]);
  }
  printf("]\n");


  // update P
  /*
  double Kt[3][3];
  trans33(ukf->K,Kt);

  printf("Kt:\n");
  for (int i =0; i < 7; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",Kt[i][j]);
    }
    printf("]\n");
  }
  
  double dot_S_Kt[3][3];
  dot33(ukf->S, Kt, dot_S_Kt);

  printf("dot_S_Kt:\n");
  for (int i =0; i < 7; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",dot_S_Kt[i][j]);
    }
    printf("]\n");
  }
  
  double dot_K_dot_S_Kt[3][3];
  dot33(ukf->K, dot_S_Kt, dot_K_dot_S_Kt);

  printf("dot_K_dot_S_Kt:\n");
  for (int i =0; i < 7; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",dot_K_dot_S_Kt[i][j]);
    }
    printf("]\n");
  }
  */

  // Joseph's stabilized covariance update
  double temp1[3][3]; // I - KH
  double temp1T[3][3];
  double temp2[3][3]; // temp1 * P_prior
  double temp3[3][3]; // temp2 * temp1^T
  double temp4[3][3]; // K * R
  double temp5[3][3]; // temp4 * K^T
  double KT[3][3];
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

  printf("x:\n");
  printf("[");
  for (int i = 0; i < 3; i++){
    printf("%.4f ",ukf->x[i]);
  }
  printf("]\n");

  printf("P:\n");
  for (int i = 0; i < 3; i++){
    printf("[");
    for (int j = 0; j < 3; j++){
      printf("%.4f ",ukf->P[i][j]);
    }
    printf("]\n");

  for (int i = 0; i < 3; i++){
    ukf->x_post[i] = ukf->x[i];
    for (int j = 0; j < 3; j++){
      ukf->P_post[i][j] = ukf->P_post[i][j];
    } // end nested for loop
  } // end for loop

  }


} // end update
