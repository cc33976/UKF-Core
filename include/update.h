#ifndef UPDATE_H
#define UPDATE_H

#include "UnscentedKalmanFilter.h"
#include "merwe.h"
#include "cross_variance.h"
#include "residual.h"
#include "trans33.h"
#include "dot33.h"
#include "dot3.h"

void update(UKF *ukf,
	    MerweSigmaPoints *sp,
	    double R[3][3],
	    double H[3][3],
	    double z[3]);

#endif
