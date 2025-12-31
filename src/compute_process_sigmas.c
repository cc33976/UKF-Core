#include "compute_process_sigmas.h"
#include "merwe.h"
#include "UnscentedKalmanFilter.h"
#include "f_func.h"
#include <stdio.h>



void compute_process_sigmas(UKF *ukf,
			    MerweSigmaPoints *sp) {

  printf("entering compute_process_sigmas\n");
  double sigmas[7][3];

  // create sigma points around given mean values x
  sigma_points(sp, sigmas, ukf->x, ukf->P);

  printf("printing sigma points from sigma_points fn:\n");
  for (int i=0; i<7; i++){
    printf("[");
    for (int j=0; j<3; j++){
      printf("%.4f ",sigmas[i][j]);
    }
    printf("]\n");
  }
  

} // end compute_process_sigmas
