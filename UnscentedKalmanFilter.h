#ifndef UNSCENTEDKALMANFILTER_H
#define UNSCENTEDKALMANFILTER_H

// forward declaration of MerweSigmaPoints struct:
typedef struct MerweSigmaPoints MerweSigmaPoints;

// forward declarations of fx and hx fn pointers
typedef void (*fx_fn)(double x[3], double dt, double x_out[3]);
typedef void (*hx_fn)(double x[3], double z_out[3]);

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

  // function pointer declarations
  fx_fn fx;
  hx_fn hx; 

  // Kalman Gain
  double K[3][3];

  // System uncertainty
  double S[3][3];
  

} UKF;

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
	      MerweSigmaPoints *sp);

// create destructor-like function for cleanup after running
void destroy_UKF(UKF *ukf);

void predict(UKF *ukf);

void update(UKF *ukf, const double z[3]); 

#endif
