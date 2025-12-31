#ifndef UNSCENTEDKALMANFILTER_H
#define UNSCENTEDKALMANFILTER_H

#include "merwe.h"

typedef struct UKF UKF;

struct MerweSigmaPoints;

// forward declarations of fx and hx fn pointers
typedef void (*fx_fn)(UKF *ukf, double x_in[3], double x_out[3]);
typedef void (*hx_fn)(double x_in[3], double H[3][3], double z_out[3]);


 struct UKF{
  int dim_x;
  int dim_z;
  double dt;
  
  struct MerweSigmaPoints *sp;

  double x[3];
  double x_prior[3];
  double x_post[3]; 

  // pointers to predefined 3x3 arrays s
  double P[3][3];
  double P_prior[3][3];
  double P_post[3][3];
  double Q[3][3];
  double R[3][3];

  // sigma points from fx and hx functions
  double sigmas_f[7][3];
  double sigmas_h[7][3];

  // residuals of state x and measurement z
  double residual_x[3];
  double residual_z[3];
  double y[3];

  // function pointer declarations
  fx_fn fx;
  hx_fn hx; 

  // Kalman Gain
  double K[3][3];

  // System uncertainty
  double S[3][3];
  double SI[3][3];
  

};


// create constructor-like object
void init_UKF(UKF *ukf,
	      int dim_x,
	      int dim_z,
	      double P[3][3],
	      double Q[3][3],
	      double R[3][3],
	      double dt,
	      fx_fn fx,
	      hx_fn hx,
	      struct MerweSigmaPoints *sp);


#endif
