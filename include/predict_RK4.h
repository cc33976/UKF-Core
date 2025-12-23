#ifndef PREDICT_RK4_H
#define PREDICT_RK4_H

#include "f.h"
#include "UnscentedKalmanFilter.h"

void predict_RK4(UKF *ukf, const double x_in[3], double y[3]);

#endif
