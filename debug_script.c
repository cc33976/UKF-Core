#include <stdio.h>
#include <math.h>
#include "cholesky.h"

void cholesky(const double A[3][3], double L[3][3]) {

    if (A[0][0] <= 0.0) {
        printf("Cholesky error: A[0][0]=%f <= 0\n", A[0][0]);
        return;
    } // end if

    /*
    printf("A:\n");
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 3; j++){
	printf("%.3f ",A[i][j]);
      }
      printf("\n");
    }
    */

    // initialize L to zeros
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            L[i][j] = 0.0;
	    // printf("%.3f ",L[i][j]);
        } // end nested for loop
	//printf("\n");
    } // end outer for loop
    

    // column by column
    L[0][0] = sqrt(A[0][0]);
    L[1][0] = A[1][0] / L[0][0];
    L[2][0] = A[2][0] / L[0][0];

    double temp = A[1][1] - L[1][0]*L[1][0];

    // prevent tiny negatives from sneaking in
    double eps = 1e-12;
    if (temp < eps){
      printf("Cholesky warning: temp to small, switching to eps\n");
      temp = eps;
    } // end if
    
    if (temp <= 0) {
        printf("Cholesky error: A[1][1]-L[1][0]^2=%f <= 0\n", temp);
        return;
    } // end if 
    L[1][1] = sqrt(temp);
    L[2][1] = (A[2][1] - L[2][0]*L[1][0]) / L[1][1];

    temp = A[2][2] - L[2][0]*L[2][0] - L[2][1]*L[2][1];
    if (temp <= 0) {
        printf("Cholesky error: A[2][2]-sum=%f <= 0\n", temp);
        return;
    } // end if 
    L[2][2] = sqrt(temp);

    /*
    // print L matrix
    printf("L:\n");
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 3; j++){
	printf("%.3f ",L[i][j]);
      }
      printf("\n");
    }
    */
} // end cholesky

#include "compute_K_cholesky.h"

// solves for the Kalman 

void forward_solve_L_3x3(const double L[3][3],
                         const double b[3],
                         double y[3])
{
    for (int i = 0; i < 3; i++) {
        double sum = b[i];
        for (int j = 0; j < i; j++) {
            sum -= L[i][j] * y[j];
        }
        y[i] = sum / L[i][i];
    }
}

void back_solve_LT_3x3(const double L[3][3],
                       const double y[3],
                       double x[3])
{
    for (int i = 2; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < 3; j++) {
            sum -= L[j][i] * x[j];  // NOTE: transpose access
        }
        x[i] = sum / L[i][i];
    }
}


void back_solve_L_3x3(const double L[3][3],
                     const double y[3],
                     double x[3])
{
    for (int i = 2; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < 3; j++) {
            sum -= L[i][j] * x[j];
        }
        x[i] = sum / L[i][i];
    }
}

void compute_K_cholesky(const double Pxz[3][3],
                        const double L[3][3],
                        double K[3][3])
{
    double b[3], y[3], x[3];

    for (int i = 0; i < 3; i++) {

        // b = row i of Pxz
        b[0] = Pxz[0][i];
        b[1] = Pxz[1][i];
        b[2] = Pxz[2][i];

        // L y = b
        forward_solve_L_3x3(L, b, y);

        // Lᵀ x = y
        back_solve_LT_3x3(L, y, x);

        // Store row of K
        K[0][i] = x[0];
        K[1][i] = x[1];
        K[2][i] = x[2];
    }
} // end compute_K_cholesky

#include "compute_process_sigmas.h"
#include "merwe.h"
#include "UnscentedKalmanFilter.h"
#include "f_func.h"
#include <stdio.h>



void compute_process_sigmas(UKF *ukf,
			    MerweSigmaPoints *sp) {

  printf("entering compute_process_sigmas\n");

  
  // generate sigma points for give state (x,P)
  double sigmas[7][3];

  // create sigma points around given mean values x
  sigma_points(sp, sigmas, ukf->x, ukf->P);

  // pass sigma points to the f_func for model state prediction
  
  for (int i=0; i < 7; i++) {
    f_func(ukf, sigmas[i], ukf->sigmas_f[i]);

  } // end for loop

  printf("sigmas_f:\n");
  for (int i = 0; i < 3; i++){
    printf("[");
    for (int j = 0; j < 3; j++){
      printf("%.4f ",ukf->sigmas_f[i][j]);
    }
    printf("]\n");
  }

  

} // end compute_process_sigmas

#include "compute_weights.h"
#include "merwe.h"

void compute_weights(MerweSigmaPoints *sp){


  double c = 0.5 / (sp->n + sp->lambda);
  for (int i=0; i < 2*sp->n + 1; i++) {
    sp->Wc[i] = c;
    sp->Wm[i] = c;
  } // end for loop

  sp->Wc[0] = sp->lambda / (sp->n + sp->lambda) +
    (1 - sp->alpha*sp->alpha + sp->beta);
  
  sp->Wm[0] = sp->lambda / (sp->n + sp->lambda);


} // end compute weights

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


#include "dot.h"

// might be a good idea to make Wm and x_new ambiguous (a,b) 
void dot(double A[7], double B[7][3], double Out[3]) {

  double sum = 0.0;
  
  for (int j=0; j < 3; j++){
    sum = 0.0;
    for (int i=0; i< 7; i++){
      sum = sum + (A[i] * B[i][j]);
    } // end nested for loop
    Out[j] = sum;
  } // end for loop

} // end dot 


#include "dot3.h"

// might be a good idea to make Wm and x_new ambiguous (a,b) 
void dot3(double m[3][3], double v[3], double out[3]) {
  
  for (int i=0; i < 3; i++){
    double sum = 0.0;
    for (int j=0; j< 3; j++){
      sum += (m[i][j] * v[j]);
    } // end nested for loop
    out[i] = sum;
  } // end for loop

} // end dot 


#include "dot33.h"

// computes the dot product of two 3x3 matrices
void dot33(double A[3][3], double B[3][3], double Out[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += A[i][k] * B[k][j];
            } // end nested for loop
            Out[i][j] = sum;
        } // end middle for loop
    } // end outermost for loop
} // end dot33

#include "f_func.h"
#include "predict_RK4.h"
#include "UnscentedKalmanFilter.h"


void  f_func(UKF *ukf, double x_in[3], double x_out[3]){
  /*
    f_func for UKF uses nonlinear  RK4 model on sigma points rather than
    linearized predictin model
  */

  // pass the points through RK4 and return
  predict_RK4(ukf, x_in, x_out);


} // end f_func

#include "f.h"

/* 
F function that returns the f(x) derivative function from EOM. Returns the
velocity v, the acceleratin a, and the jerk j (assumed 0 at small dt)
*/ 

void f(double x[3], double dxdt[3]){
  
  // x is 2D, 3xN matrix holding p, v, a. One row passed @ a time making 1x3.
  dxdt[0] = x[1];
  dxdt[1] = x[2];
  dxdt[2] = 0.0;
  
} // end f

#include "h_func.h"
#include "UnscentedKalmanFilter.h"



void h_func(double x_in[3], double H[3][3], double z_out[3]){

  double tot_row; 

  // H is a 3x3 matrix
  // x is a 3x1 matrix
  for (int i=0; i < 3; i++){
    tot_row = 0.0;
    for (int j=0; j < 3; j++){
      
      // find sum product for each row and add to row index of product matrix
      tot_row += H[i][j] * x_in[j];

    } // end inner for (col)
    
    z_out[i] = tot_row;
  } // end outer for (row)

} // end h_func


#include "load_csv.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_ROWS 2000

int load_csv(const char *filename, double Z[][3]) {
  
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    perror("Failed to open launch data\n");
    return -1; 
  } // end if

  char line[256];
  int row = 0;

  while (fgets(line, sizeof(line), fp) && row < MAX_ROWS) {
    sscanf(line, "%lf, %lf, %lf",
	   &Z[row][0],
	   &Z[row][1],
	   &Z[row][2]);
    row++; 
  } // end while

  fclose(fp);
  return row; // number of rows scanned in 

} // end load_csv


#include "residual.h"
#include <stdio.h>

void residual(double a[3], double b[3], double out[3]) {

  printf("#### entering residual ####\n");

  printf("a: ");
  for (int i=0;i<3;i++){
    printf("%.4f ",a[i]);
  } // end for
  printf("\n");

  printf("b: ");
  for (int i=0;i<3;i++){
    printf("%.4f ",b[i]);
  } // end for
  printf("\n");

  for (int i=0; i < 3; i++){
    out[i] = b[i] - a[i];
  } // end for loop

  printf("out: ");
  for (int i=0;i<3;i++){
    printf("%.4f ",out[i]);
  } // end for
  printf("\n");

  printf("#### leaving residual ####\n");

} // end residual 

#include "subtract.h"
#include <stdio.h>

/*
  The function is called subtract to remain synonymous with
  the python UKF function names, but should be called
  'distribute' as it adds three sigmas above and below the
  average value.
*/ 

void subtract(double x[3], double U[3][3], double sigmas[7][3]){

  /*
  printf("Entering the subtract function\n");
  printf("printing inputs:\n");
  printf("x:\n");
  printf("[");
  for (int i=0;i<3;i++){
    printf("%.3f ",x[i]);
  }
  printf("]\n");

  printf("U:\n");
  for (int i=0;i<3;i++){
    printf("[");
    for (int j=0;j<3;j++){
      printf("%.3f ",U[i][j]);
    }
    printf("]\n");
  }
  */
  

  for (int i = 0; i < 3; i++){
    for (int j=0; j < 3; j++){
      // transpose of U because is lower triangular
      sigmas[i+1][j] = x[j] - U[j][i];
      //printf("lower sigma (-): %.4f\n",sigmas[i+1][j]);
      sigmas[i+4][j] = x[j] + U[j][i];
      //printf("upper sigma (+): %.4f\n",sigmas[i+4][j]);
    } // end nested for loop
  } // end outer for loop


} // end subtract 

#include "trans33.h"

void trans33(double A[3][3], double At[3][3]) {
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++) {
      At[j][i] = A[i][j]; 
    } // end nested for loop
  } // end outer for loop 

} // end trans 

#include "unscented_transform.h"
#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "residual.h"
#include "unscented_transform.h"
#include "dot.h"
#include <stdio.h>

// returns state and covariance matrices (x,P)
void unscented_transform(double sigmas[7][3],
			 double Wm[7],
			 double Wc[7],
			 double noise_cov[3][3],
			 UKF *ukf,
			 double x[3],
			 double P[3][3]){

  printf("#### entering unscented transform ####\n");

  printf("sigmas_f:\n");
  for (int i =0; i < 7; i++){
    printf("[");
    for (int j=0; j< 3; j++){
      printf("%.4f ",sigmas[i][j]);
    }
    printf("]\n");
  }

  // set P to zero before accumulating
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      P[i][j] = 0.0;
    } // end for
  } // end for 

  // compute new average state by taking weighted sum of sigma points
  dot(Wm, sigmas, x);

  // compute new covariance matrix
  double y[3] = {0};
  for (int k = 0; k < 7; k++) {
    residual(sigmas[k], x, y);  // y = sigmas[k] - x
    
    // P += Wc[k] * outer(y, y)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
	P[i][j] += Wc[k] * y[i] * y[j];
		
      } // end inner for loop 
    } // end middle for loop
  } // end k for loop

 
  // use R as noise_cov 
  if (!(noise_cov == NULL)) {
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 3; j++){
	P[i][j] += noise_cov[i][j];
      } // end nested for loop
    } // end for loop
  } // end if statement

  printf("leaving unscented transform\n");
 

} // end unscented_transform

#include "predict_RK4.h"
#include "f.h"
#include "UnscentedKalmanFilter.h"
#include <stdio.h>

// function for computing Runge-Kutta 4 for robust prediction methods in UKF
void predict_RK4(UKF *ukf, double x[3], double y[3]){

  printf("\n");
  printf("entering RK4\n");

  double dt = ukf->dt;
  
  // declare and allocate 1x3 for k values
  double k1[3];
  double k2[3];
  double k3[3];
  double k4[3];
  double x_new[3];

  f(x,k1);
  printf("k1: [");
  for (int i=0; i<3; i++){
    printf("%.4f ",k1[i]);
  }
  printf("]\n");

  // create new x to pass for k2 
  for (int i=0; i < 3; i++){
    x_new[i] = x[i] + 0.5 * k1[i];
  } // end for

  f(x_new,k2);
  printf("k2: [");
  for (int i=0; i<3; i++){
    printf("%.4f ",k2[i]);
  }
  printf("]\n");

  // create new x to pass for k3 
  for (int i=0; i < 3; i++){
    x_new[i] = x[i] + 0.5 * k2[i];
  } // end for

  f(x_new,k3);
  printf("k3: [");
  for (int i=0; i<3; i++){
    printf("%.4f ",k3[i]);
  }
  printf("]\n");

  // create new x to pass for k4

  for (int i=0; i < 3; i++){
    x_new[i] = x[i] + k3[i];
  } // end for

  f(x_new,k4);
  printf("k4: [");
  for (int i=0; i<3; i++){
    printf("%.4f ",k4[i]);
  }
  printf("]\n");

  printf("y: [");
  // calculate final prediction
  for (int i=0; i < 3; i++){
    y[i] = x[i] + dt/6.0 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    printf("%.4f ",y[i]);
  }
  printf("]\n");

  // prediction is a 1x3

} // end predict_RK4


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
  

  // compute sigma points and process through RK4 for sigmas_f
  compute_process_sigmas(ukf, sp);
 

  // pass sigmas through unscented transform to calculate prior
  unscented_transform(ukf->sigmas_f, sp->Wm, sp->Wc, ukf->Q, ukf, ukf->x, ukf->P);

  printf("back in predict\n");

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


} // end predict


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

  printf("Print weights for debugging\n");
  printf("Lambda = %.6f\n", sp->lambda);
  printf("Weights (mean): ");
  for (int i = 0; i < 2*sp->n + 1; i++) {
    printf("%.6f ", sp->Wm[i]);
  }
  printf("\nWeights (cov): ");
  for (int i = 0; i < 2*sp->n + 1; i++) {
    printf("%.6f ", sp->Wc[i]);
  }
  printf("\n");
  
} // end merweCreate




void sigma_points(MerweSigmaPoints *sp,
		  double sigmas[7][3],
		  double x[3],
		  double P[3][3]){

  int n = sp->n;
  double lambda = sp->lambda;

  printf("entering sigma points\n");
  //printf("n = %d, lambda = %.3f\n",n,lambda);

  // need access to cholesky here: U = self.sqrt((lambda_ + n)*P)
  double P_new[3][3];
  
  for (int i=0; i < 3; i++){
    for (int j=0; j < 3; j++){
      P_new[i][j] = P[i][j] * (lambda + n);
    } // end inner for loop
  } // end outer for loop

  printf("New P matrix:\n");
  for (int i=0; i<3;i++){
    printf("[");
    for (int j=0;j < 3; j++){
      printf("%.3f ",P_new[i][j]);
    }
    printf("]\n");
  }
  
  double U[3][3];
  double temp[3][3];

  // perform cholesky for U 
  for (int i = 0; i < 3; i++){
    for (int j=0; j < 3; j++){
      temp[i][j] = (lambda + n)*P[i][j];
    } // end nested for loop
  } // end outer for loop

  cholesky(temp, U);

 
  printf("print the new U matrix:\n");
  for (int i=0; i<3;i++){
    printf("[");
    for (int j=0;j < 3; j++){
      printf("%.3f ",U[i][j]);
    }
    printf("]\n");
  }

  
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

  double K_temp[3][3] = {0};
  compute_K_cholesky(Pxz,L_SI,K_temp); // computes transposed K
  trans33(K_temp, ukf->K); // transpose back into K and store 

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
  dot3(ukf->K, ukf->y, temp_x); // was y,K =============###########
  
  printf("temp_x:\n");
  printf("[");
  for (int i = 0; i < 3; i++){
    printf("%.4f ",temp_x[i]);
  }
  printf("]\n");



  // Joseph's stabilized covariance update P
  double temp1[3][3]; // I - KH
  double temp1T[3][3]; // temp1^T
  double temp2[3][3]; // temp1 * P_prior
  double temp3[3][3]; // temp2 * temp1^T
  double temp4[3][3]; // K * R
  double temp5[3][3]; // temp4 * K^T
  double KT[3][3]; // K^T
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
  }
  

  for (int i = 0; i < 3; i++){
    ukf->x_post[i] = ukf->x[i];
    for (int j = 0; j < 3; j++){
      ukf->P_post[i][j] = ukf->P_post[i][j];
    } // end nested for loop
  } // end for loop



} // end update


#include "UnscentedKalmanFilter.h"
#include <stdio.h>
#include <stdlib.h>


// create UKF constructor-like struct

void init_UKF(UKF *ukf,
	      int dim_x,
	      int dim_z,
	      double x[3],
	      double P[3][3],
	      double Q[3][3],
	      double R[3][3],
	      double dt,
	      fx_fn fx,
	      hx_fn hx,
	      struct MerweSigmaPoints *sp) {

  printf("Entered the init_UKF function\n");

  ukf->x[0] = x[0]; // [m]
  ukf->x[1] = x[1]; // [m/s]
  ukf->x[2] = x[2]; // [m/s^2]

  
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++){
      ukf->P[i][j] = P[i][j];
      ukf->Q[i][j] = Q[i][j];
      ukf->R[i][j] = R[i][j];
    } // end nested for loop
  } // end outer for loop

  //printf("Successfully allocated P,Q,R matrices to UKF 'object'\n");

  //printf("Attempting to assign sp and dt values in UKF 'object'\n");
  ukf->sp = sp;
  ukf->dt = dt;
  //printf("Assigned sp and dt values\n");

  //printf("Attempting to assign fx and hx function pointers in UKF 'object'\n");
  // assign function pointers 
  ukf->fx = fx;
  ukf->hx = hx;
  //printf("Assigned fx and hx function pointers\n");

  
  
} // end init_UKF


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h> 
#include "f_func.h"
#include "h_func.h"
#include "merwe.h"
#include "predict_RK4.h"
#include "UnscentedKalmanFilter.h"
#include "load_csv.h"
#include "predict.h"
#include "update.h"
#include "unscented_transform.h"
#include "f_func.h"
#include "h_func.h" 



int main(void){
  // argc -- num command line arguments
  // argv -- array of strings containing arguments

  double zs[157][3];
  //double xs_ukf[157][3];
  //double Ps_ukf[157][3];
  double z[3];

  // import flight data into z matrix (measurements)
  const char filename[256] = "/Users/casey/Desktop/Kalman_Test_Data/Kalman_Test_data.csv";
  load_csv(filename, zs);

  /*
  printf("zs:\n");
  for (int i = 0; i < 157; i++){
    for (int j = 0; j < 3; j++){
      printf("%.3f ",zs[i][j]);
    }
	printf("\n");
  }
  */
  

  // constant declarations:
  double var_alt = 1.0;
  double var_vel = 2.0;
  double var_accel = 1.0;
  double rho_R = 0.7; // for covariance weights
  double rho_Q = 0.7; // for white noise weights
  double dt = 0.117; // seconds

  // initial guess 
  double x_init[3] = {0.2, 0.1, 9.8};

  // create a 3x3 I matrix for H
  double H[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };

  // create 3x3 diagonal matrix for measurement variances
  double R[3][3] = {
    {var_alt, 0, 0},
    {0, var_vel, 0},
    {0, 0, var_accel}
  };

  
  // create initial covariance matrix
  double P0[3][3] = {
    {9*var_alt, rho_R*sqrt(var_alt*var_vel), rho_R*sqrt(var_alt*var_accel)},
    {rho_R*sqrt(var_vel*var_alt), 9*var_vel, rho_R*sqrt(var_vel*var_accel)},
    {rho_R*sqrt(var_alt*var_accel), rho_R*sqrt(var_vel*var_accel), 9*var_accel}
  };
  
  // create white noise matrix for process noise
  double Q_UKF[3][3] = {
    {var_alt, rho_Q*sqrt(var_alt*var_vel), rho_Q*sqrt(var_alt*var_accel)},
    {rho_Q*sqrt(var_vel*var_alt), var_vel, rho_Q*sqrt(var_vel*var_accel)},
    {rho_Q*sqrt(var_alt*var_accel), rho_Q*sqrt(var_vel*var_accel), var_accel},
  };
  

  // =============== UKF SETUP ===============
  // create sigma points 
  int n = 3;
  double alpha = 0.5;
  double beta = 2.0;
  double kappa = 0.0; 
  MerweSigmaPoints sp;
  //printf("Attempting to create sigma points\n");
  merweCreate(&sp, n, alpha, beta, kappa);
  //printf("successfully created sigma points\n");


  // create UKF
  UKF ukf;
  //printf("Attempting to create UKF 'object'\n");
  init_UKF(&ukf, 3, 3, x_init, P0, Q_UKF, R, dt, f_func, h_func, &sp);
  //printf("created UKF 'object with init_UKF'\n");

  

  // ============== filter loop ================
  for (int i = 0; i < 157; i++){
    for (int j = 0; j < 3; j++){ 
      z[j] = zs[i][j];
    } // end nested for loop

    for (int i=0; i<3; i++){
      printf("z[%d]: ",i);
      printf("%.4f ",z[i]);
    }
    printf("]\n");	   

    printf("Attempting predict step in loop iteration: %d\n",i);
    // predict step
    predict(&ukf, &sp);
    printf("\n\n");


    
    printf("entered back into main filter loop\n");
    printf("Attempting update step in loop iteration %d\n",i);
    // update step
    update(&ukf, &sp, R, H, z);
    printf("\n======================================================\n");

    
  } //========== end main batch filter loop=========

  

  // create time array for data plotting
  double time[157];
  double temp = 0.0;
  for (int i = 0; i < 157; i++) {
    time[i] = temp;
    temp += dt;     
  } // end for loop

 
  printf("alt: %f [m]\n",ukf.x[0]);
  printf("vel: %f [m/s]\n",ukf.x[1]);
  printf("accel: %f [m/s^2]\n",ukf.x[2]);



} // end main 


