#ifndef UNSCENTEDKALMANFILTER_H
#define UNSCENTEDKALMANFILTER_H

// forward declaration of MerweSigmaPoints struct:
typedef struct MerweSigmaPoints MerweSigmaPoints;

typedef struct {
  int dim_x;
  int dim_z;
  double dt;
  
  MerweSigmaPoints *sp;

  double x[3];
  double x_prior[3];

  // pointers to predefined 3x3 arrays s
  double (*P)[3];
  double P_prior[3][3];
  double (*Q)[3];
  double (*R)[3];

  // intitialize the weights
  double Wm;
  double Wc;

  // Kalman Gain
  double K[3][3];

  // System uncertainty
  double S[3][3];
  

} UKF;


// create constructor-like object
void UKF(UKF *ukf, int dim_x, double P[3][3], double Q[3][3], double R[3][3], double dt, MerweSigmaPoints *sp);

// create destructor-like function for cleanup after running
void destroy_UKF(UKF *ukf);

#endif
