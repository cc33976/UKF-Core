#ifndef COMPUTE_PROCESS_SIGMAS_H
#define COMPUTE_PROCESS_SIGMAS_H

#include "UnscentedKalmanFilter.h"

void compute_process_sigmas(UKF *ukf, MerweSigmaPoints *sp, double sigmas[7][3]);

#endif
