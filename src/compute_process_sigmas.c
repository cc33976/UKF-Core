#include "compute_process_sigmas.h"
#include "merwe.h"
#include "UnscentedKalmanFilter.h"
#include "f_func.h"
#include <stdio.h>



void compute_process_sigmas(UKF *ukf,
			    MerweSigmaPoints *sp) {

  
  // generate sigma points for give state (x,P)
  double sigmas[7][3] = {0};

  // create sigma points around given mean values x
  sigma_points(sp, sigmas, ukf->x, ukf->P);

  // pass sigma points to the f_func for model state prediction
  
  for (int i=0; i < 7; i++) {
    f_func(ukf, sigmas[i], ukf->sigmas_f[i]);

  } // end for loop
  

} // end compute_process_sigmas
