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
#include "save_ukf_to_csv.h"



int  main(void){
  // argc -- num command line arguments
  // argv -- array of strings containing arguments

  double zs[157][3];
  double xs_ukf[157][3];
  //double Ps_ukf[157][3];
  double z[3];

  // import flight data into z matrix (measurements)
  const char filename[256] = "/Users/casey/Desktop/Kalman_Test_Data/Kalman_Test_data.csv";
  load_csv(filename, zs);
  

  // constant declarations:
  double var_alt = 1.0;
  double var_vel = 2.0;
  double var_accel = 1.0;
  double rho_R = 0.7; // for covariance weights
  double rho_Q = 0.8; // for white noise weights
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
  merweCreate(&sp, n, alpha, beta, kappa);


  // create UKF
  UKF ukf;
  init_UKF(&ukf, 3, 3, x_init, P0, Q_UKF, R, dt, f_func, h_func, &sp);
  

  // ============== filter loop ================
  for (int i = 0; i < 157; i++){
    for (int j = 0; j < 3; j++){ 
      z[j] = zs[i][j];
    } // end nested for loop	   

    // predict step
    predict(&ukf, &sp);

    // update step
    update(&ukf, &sp, R, H, z);

    for (int l = 0; l < 3; l++){
      xs_ukf[i][l] = ukf.x[l];
    }
    //printf("%.4f [m], %.4f [m/s], %.4f [m/s^2]\n",xs_ukf[i][0],xs_ukf[i][1], xs_ukf[i][2]);

  } //========== end main batch filter loop=========

  

  // create time array for data plotting
  double time[157];
  double temp = 0.0;
  for (int i = 0; i < 157; i++) {
    time[i] = temp;
    temp += dt;     
  } // end for loop

  printf("============================================\n");
  printf("alt: %f [m]\n",ukf.x[0]);
  printf("vel: %f [m/s]\n",ukf.x[1]);
  printf("accel: %f [m/s^2]\n",ukf.x[2]);
  printf("============================================\n\n");


  // write to output file
  const char outfilename[256] = "/Users/casey/Desktop/UnscentedKalmanFilter/ukf_output.csv";
  int rows = 157;
  save_ukf_to_csv(outfilename, time, xs_ukf, rows);

} // end main 
