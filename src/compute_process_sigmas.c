#include "compute_process_sigmas.h"
#include "merwe.h"
#include "UnscentedKalmanFilter.h"
#include "f_func.h" 



void compute_process_sigmas(UKF *ukf,
			    MerweSigmaPoints *sp,
			    double sigmas[7][3]) {

  double temp_sigmas[7][3];

  // create sigma points around given mean values x
  sigma_points(ukf, sp, temp_sigmas, ukf->x, ukf->P);


  // pass sigma points to the f_func for model state prediction
  
  for (int i=0; i < 7; i++) {
    f_func(ukf, temp_sigmas[i], ukf->sigmas_f[i]);

  } // end for loop

} // end compute_process_sigmas
