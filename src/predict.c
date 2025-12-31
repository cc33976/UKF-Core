#include "predict.h" 
#include "UnscentedKalmanFilter.h"
#include "compute_process_sigmas.h"
#include "merwe.h"
#include <stdio.h>
#include "f_func.h"
#include "unscented_transform.h"

void predict(UKF *ukf,
	     MerweSigmaPoints *sp){

  printf("Entering the predict function\n");
  
  // intialize the priors
  for (int i = 0; i < 3; i++){
    ukf->x_prior[i] = ukf->x_post[i];
    for (int j = 0; j < 3; j++){
      ukf->P_prior[i][j] = ukf->P_post[i][j];
    } // end nested for loop
  } // end for loop

  printf("x_prior:\n [");
  for (int i = 0; i < 3; i++){
    printf("%.4f ",ukf->x_prior[i]);
  } 
  printf("]\n");

  printf("P_prior:\n");
  for (int i = 0; i < 3; i++){
    printf("[");
    for (int j = 0; j < 3; j++){
    printf("%.4f ",ukf->P_prior[i][j]);
    }
    printf("]\n");
  }

      

  double sigmas[7][3];
  double sigmas_f[7][3];

  sigma_points(sp, sigmas, ukf->x_prior, ukf->P_prior);


  // pass sigma points to the f_func for model state prediction
  
  for (int i=0; i < 7; i++) {
    f_func(ukf, sigmas[i], sigmas_f[i]);

  } // end for loop

  
  // =========print the sigma points for check========
  printf("sigmas_f:\n");
  for (int i =0; i < 7; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",sigmas_f[i][j]);
    }
    printf("]\n");
  }
 

  // pass sigmas through unscented transform to calculate prior
  unscented_transform(sigmas_f, sp->Wm, sp->Wc, ukf->Q, ukf, ukf->x, ukf->P);

  printf("back in predict\n");
  printf("sigmas_f:\n");
  for (int i =0; i < 7; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",sigmas_f[i][j]);
    }
    printf("]\n");
  }

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

  // copy sigmas_f to ukf->sigmas_f ===== fix later. should be using struct
  for (int i =0; i < 7; i++){
    for (int j=0; j< 3; j++){
      ukf->sigmas_f[i][j] = sigmas_f[i][j];
    }
  }
 


} // end predict
