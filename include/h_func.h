#ifndef H_FUNC_H
#define H_FUNC_H

#include "UnscentedKalmanFilter.h"

void h_func(double x_in[3], double H[3][3], double z_out[3]);

#endif
