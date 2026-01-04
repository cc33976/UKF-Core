#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "cholesky.h"
#include "subtract.h"
#include <math.h>
#include <stdio.h> 

void merweCreate(MerweSigmaPoints *sp,
		 int n,
		 double alpha,
		 double beta,
		 double kappa){


  // assign sp params from function input
  sp->n = n;
  sp->alpha = alpha;
  sp->beta = beta;
  sp->kappa = kappa;
  sp->lambda = alpha*alpha * (n + kappa) - n;

  // compute weights
  sp->Wm[0] = sp->lambda / ((double)n + sp->lambda);
  sp->Wc[0] = sp->Wm[0] + (1.0 - alpha*alpha + beta);

  for (int i = 1; i < 2*n + 1; i++){
    sp->Wm[i] = 1.0 / (2.0*((double)n+sp->lambda));
    sp->Wc[i] = sp->Wm[i];
  }

  
} // end merweCreate




void sigma_points(MerweSigmaPoints *sp,
		  double sigmas[7][3],
		  double x[3],
		  double P[3][3]){

  int n = sp->n;
  double lambda = sp->lambda;


  // need access to cholesky here: U = self.sqrt((lambda_ + n)*P)
  double P_new[3][3];
  
  for (int i=0; i < 3; i++){
    for (int j=0; j < 3; j++){
      P_new[i][j] = P[i][j] * (lambda + n);
    } // end inner for loop
  } // end outer for loop
  
  double U[3][3];
  double temp[3][3];

  // perform cholesky for U 
  for (int i = 0; i < 3; i++){
    for (int j=0; j < 3; j++){
      temp[i][j] = (lambda + n)*P[i][j];
    } // end nested for loop
  } // end outer for loop

  cholesky(temp, U);

  
  // assign sigmas matrix to zeros
  for (int i=0; i<7; i++){
    for (int j=0; j < 3; j++){
      sigmas[i][j] = 0.0; 
    }// end nested for loop
  }// end outer for loop

  // assign the first row of sigmas to the x values
  for (int i = 0; i < 3; i++){ 
    sigmas[0][i] = x[i];
  }

  // finish the rest of the sigma points
  subtract(x, U, sigmas);

} // end sigma_points




