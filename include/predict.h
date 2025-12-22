#ifndef PREDICT_H
#define PREDICT_H

void (*fx)(double x[3], double dt, double x_out[3]);

void predict(); // end predict



#endif
