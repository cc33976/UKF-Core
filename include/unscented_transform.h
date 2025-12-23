#ifndef UNSCENTED_TRANSFORM_H
#define UNSCENTED_TRANSFORM_H

#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "residual.h"

void unscented_transform(double sigmas[7][3],
			 double Wm[7],
			 double Wc[7],
			 double noise_cov[3][3],
			 UKF *ukf,
			 double x[3],
			 double P[3][3]);

#endif 


