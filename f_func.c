#include <stdio.h>
#include "predict_RK4.h"


double** f_func(double** x, double dt){
  /*
    f_func for UKF uses nonlinear  RK4 model on sigma points rather than
    linearized predictin model
  */

  // pass the points through RK4 and return
  return predict_RK4(x, dt);


} // end f_func

