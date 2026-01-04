#include "predict.h" 
#include "UnscentedKalmanFilter.h"
#include "compute_process_sigmas.h"
#include "merwe.h"
#include <stdio.h>
#include "f_func.h"
#include "unscented_transform.h"

void predict(UKF *ukf,
	     MerweSigmaPoints *sp){
  

  // compute sigma points and process through RK4 for sigmas_f
  compute_process_sigmas(ukf, sp);
 

  // pass sigmas through unscented transform to calculate prior
  unscented_transform(ukf->sigmas_f, sp->Wm, sp->Wc, ukf->Q, ukf, ukf->x, ukf->P);


  // CLAMP negative P 
  for (int i = 0; i < 3; i++){
    if (ukf->P[i][i] < 1e-12) ukf->P[i][i] = 1e-12;
  }

  // save prior 
  for (int i = 0; i < 3; i++) {
    ukf->x_prior[i] = ukf->x[i];
    for (int j = 0; j < 3; j++) {
      ukf->P_prior[i][j] = ukf->P[i][j];

    } // end nested for loop
  } // end for loop

} // end predict
