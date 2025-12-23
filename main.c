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
  

  // constant declarations:
  double var_alt = 3.0;
  double var_vel = 1.0;
  double var_accel = 0.1;
  double rho_R = 0.7; // for covariance weights
  double rho_Q = 0.7; // for white noise weights
  double dt = 0.117; // seconds 

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
    {0, 0, var_alt}
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
  double sigmas[n][2*n+1];
  merweCreate(&sp, n, alpha, beta, kappa);


  // create UKF -- head to UnscentedKalmanFilter.c 
  UKF ukf;
  init_UKF(&ukf, 3, 3, P0, Q_UKF, R, dt, f_func, h_func, &sp);

  // ============== filter loop ================


  for (int i = 0; i < 157; i++){
    for (int j = 0; j < 3; j++){ 
      z[i] = zs[i][j];
    } // end nested for loop

    // predict step
    predict(&ukf, &sp, sigmas, unscented_transform);

    // update step
    update(&ukf, &sp, R, unscented_transform, H, z);

    // append x's to the filtered array xs_ukf

  } // end main batch filter loop


} // end main 
