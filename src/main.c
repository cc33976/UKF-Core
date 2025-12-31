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
  

  /*
  double P0[3][3] = {
    {var_alt, 0, 0},
    {0, var_vel, 0},
    {0, 0, var_accel}};

  */

  // create white noise matrix for process noise
  
  double Q_UKF[3][3] = {
    {var_alt, rho_Q*sqrt(var_alt*var_vel), rho_Q*sqrt(var_alt*var_accel)},
    {rho_Q*sqrt(var_vel*var_alt), var_vel, rho_Q*sqrt(var_vel*var_accel)},
    {rho_Q*sqrt(var_alt*var_accel), rho_Q*sqrt(var_vel*var_accel), var_accel},
  };
  

  /*
  double Q_UKF[3][3] = {
    {10, 0, 0},
    {0, 10, 0},
    {0, 0, 10}};

  */

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
  init_UKF(&ukf, 3, 3, P0, Q_UKF, R, dt, f_func, h_func, &sp);
  //printf("created UKF 'object with init_UKF'\n");

  

  // ============== filter loop ================
  for (int i = 0; i < 157; i++){
    for (int j = 0; j < 3; j++){ 
      z[j] = zs[i][j];
    } // end nested for loop

    printf("Attempting predict step in loop iteration %d\n",i);
    // predict step
    predict(&ukf, &sp);
    printf("\n\n");

    printf("entered back into main filter loop\n");
    printf("sigmas_f:\n");
    for (int i =0; i < 7; i++){
      printf("[");
      for (int j=0; j< 3; j++){
	printf("%.4f ",ukf.sigmas_f[i][j]);
      }
      printf("]\n");
    }

    printf("Attempting update step in loop iteration %d\n",i);
    // update step
    update(&ukf, &sp, R, H, z);
    printf("\n ======================================================\n");

    
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
