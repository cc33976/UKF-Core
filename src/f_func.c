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

