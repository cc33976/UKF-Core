#ifndef MERWE_H
#define MERWE_H

typedef struct {
  int n;
  double alpha;
  double beta;
  double kappa;
  double lambda;
} MerweSigmaPoints;

MerweSigmaPoints(int n, double alpha, double beta, double kappa);

#endif
