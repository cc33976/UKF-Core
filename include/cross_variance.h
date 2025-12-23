#ifndef CROSS_VARIANCE_H
#define CROSS_VARIANCE_H

#include "UnscentedKalmanFilter.h"

void cross_variance(UKF *ukf,
		    MerweSigmaPoints *sp,
		    double z[3],
		    double Pxz[3][3]);

#endif
