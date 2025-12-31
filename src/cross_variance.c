#include "cross_variance.h"
#include "UnscentedKalmanFilter.h"
#include "residual.h"
#include <stdio.h>

void cross_variance(UKF *ukf,
		    MerweSigmaPoints *sp,
		    double zp[3],
		    double Pxz[3][3]){

  printf("#### entering cross variance ####\n");

  printf("zp: ");
  for (int i=0; i<3; i++){
    printf("%.4f ",zp[i]);
  }
  printf("\n");

  // compute cross variance of the state x and measurement z
  double dx[3];
  double dz[3];

  // set cross variance to zero
  for (int r=0; r<3; r++){
    for (int c=0; c<3; c++){
      Pxz[r][c] = 0.0;
	} // end nested for loop (c)
  } // end outer for loop (r)


  printf("debugging dx and dz inputs\n");
  printf("sigmas_f:\n");
  for (int i=0; i<3; i++){
    printf("[");
    for (int j=0; j<3; j++){
      printf("%.4f ",ukf->sigmas_f[i][j]);
    }
    printf("]\n");
  }

  printf("ukf->x_prior:\n [");
  for (int i=0; i<3; i++){
    printf("%.4f ",ukf->x_prior[i]);
  }
  printf("]\n");


  printf("sigmas_h:\n");
  for (int i=0; i<3; i++){
    printf("[");
    for (int j=0; j<3; j++){
      printf("%.4f ",ukf->sigmas_h[i][j]);
    }
    printf("]\n");
  }

  printf("zp:\n [");
  for (int i=0; i<3; i++){
    printf("%.4f ",zp[i]);
  }
  printf("]\n");

  // calculate residuals and add to cross variance
  for (int i = 0; i < 7; i++){
    residual(ukf->sigmas_f[i], ukf->x_prior, dx); // was ukf->x
    residual(ukf->sigmas_h[i], zp, dz);

    printf("dx[%d]: ",i);
    for (int l=0; l < 3; l++){
      printf("%.4f ",dx[l]);
    }
    printf("\n");

    printf("dz[%d]: ",i);
    for (int l=0; l < 3; l++){
      printf("%.4f ",dz[l]);
    }
    printf("\n");

    for (int r=0; r<3; r++){
      for (int c=0; c<3; c++){
	Pxz[r][c] += sp->Wc[i] * dx[r] * dz[c];
      } // end innermost for loop (c)
    } // end middle for loop (r)
  } // end for loop

  
} // end cross_variance
