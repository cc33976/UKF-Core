#ifndef PREDICT_H
#define PREDICT_H

#include "UnscentedKalmanFilter.h"
#include "compute_process_sigmas.h"
#include "merwe.h"

void (*UT)(double[7][3], double[7], double[7], double[3][3],
			double[3], double[3][3]);

void predict(UKF *ukf, MerweSigmaPoints *sp, double sigmas[7][3],
	     void (*UT)(double[7][3], double[7], double[7], double[3][3],
			UKF *, double[3], double[3][3])); 



#endif
