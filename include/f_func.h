#ifndef F_FUNC_H
#define F_FUNC_H

#include "predict_RK4.h"
#include "UnscentedKalmanFilter.h"

void f_func(UKF *ukf, const double x_in[3], double x_out[3]);

#endif
	    
