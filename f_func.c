#include "predict_RK4.h"


void  f_func(double x[3], double dt, double x_out[3]){
  /*
    f_func for UKF uses nonlinear  RK4 model on sigma points rather than
    linearized predictin model
  */

  // pass the points through RK4 and return
  predict_RK4(x, dt, x_out);


} // end f_func

