#ifndef MERWE_H
#define MERWE_H

typedef void (*sqrt_fn)(const double A[3][3], double L[3][3]); 

typedef struct {
  int n;
  double alpha;
  double beta;
  double kappa;
  double lambda;
  double Wc[2*n + 1];
  double Wm[2*n + 1];
  sqrt_fn sqrt;
} MerweSigmaPoints;

MerweSigmaPoints(int n, double alpha, double beta, double kappa);

#endif
