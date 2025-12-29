#ifndef MERWE_H
#define MERWE_H

#include "cholesky.h"
#include "subtract.h"
#include <math.h>

struct UKF;

typedef void (*sqrt_fn)(const double A[3][3], double L[3][3]); 

typedef struct MerweSigmaPoints{
  int n;
  double alpha;
  double beta;
  double kappa;
  double lambda;
  double Wc[7]; // was 2*n+1 but compiler didn't like 
  double Wm[7];
  sqrt_fn sqrt;
} MerweSigmaPoints;


void merweCreate(MerweSigmaPoints *sp,
		 int n,
		 double alpha,
		 double beta,
		 double kappa);

void sigma_points(struct UKF *ukf,
		  MerweSigmaPoints *sp,
		  double sigmas[7][3],
		  double x[3],
		  double P[3][3]);

#endif
