#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "cholesky.h"
#include "subtract.h"
#include <math.h>

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
  
}



// 1D or 2D array? 
void sigma_points(UKF *ukf,
		  MerweSigmaPoints *sp,
		  double sigmas[7][3],
		  double x[3],
		  double P[3][3]){

  int n = sp->n;
  double lambda = sp->lambda;

  // need access to cholesky here: U = self.sqrt((lambda_ + n)*P)
  double P_new[3][3];
  
  for (int i=0; i < 3; i++){
    for (int j=0; j < 3; j++){
      P_new[i][j] = ukf->P[i][j] * (lambda + n);
    } // end inner for loop
  } // end outer for loop
  
  double U[3][3];
  double temp[3][3];

  // perform cholesky for U 
  for (int i = 0; i < 3; i++){
    for (int j=0; j < 3; j++){
      temp[i][j] = (lambda + n)*ukf->P[i][j];
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
    sigmas[0][i] = x[0];
  }

  // finish the rest of the sigma points
  subtract(ukf->x, U, sigmas);

} // end sigma_points




