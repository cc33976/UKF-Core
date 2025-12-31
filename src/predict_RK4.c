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
