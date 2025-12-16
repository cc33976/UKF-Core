#include "merwe.h"
#include "zeros.h"
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <math.h>

MerweSigmaPoints merweCreate(int n, double alpha, double beta, double kappa){
  // create sigma points "object"
  MerweSigmaPoints sp;

  // assign sp params from function input
  sp.n = n;
  sp.alpha = alpha;
  sp.beta = beta;
  sp.kappa = kappa;
  sp.lambda = alpha*alpha * (n + kappa) - n;

  // return SigmaPoints "object" sp with assigned params
  return sp;
}



// 1D or 2D array? 
double** sigma_points(MerweSigmaPoints sp, double* x, double* P){
  

  // check that array length matches n:
  int array_len = sizeof(x) / sizeof(x[0]);
  if (n != array_len){
    printf("Expected array length of %d, received array length %d.\n",n,array_len);
    // add script termination here ================================================================= *****
    
  } // end if

  int n = sp.n;
  double lambda = sp.alpha**2  * (n + sp.kappa) - n;

  // need access to cholesky here: U = self.sqrt((lambda_ + n)*P)
  double** P_new[3][3];
  for (int i=0; i < 3; i++){
    for (int j=0; j < 3; j++){
      P_new[i][j] = P[i][j] * (lambda + n);
    } // end inner for loop
  } // end outer for loop
  
  double** U[3][3] = gsl_linalg_cholesky_decomp(P_new);

  // need to create a zeros() function 

} // end sigma_points




int num_sigmas(MerweSigmaPoints sp){

  return 2*sp.n + 1; 
} // end num_sigmas


