#include <stdio.h>
#include "f.h" 

/*
f_tf is a function similar to f function for RK4 prediction,
but is capable of taking larger inputs (3x7 sigma points) rather
than a 1D array.
*/

void f_tf(double X[3][7], double Y[3][7]) {

  // move each term up one row for derivate (j=0 in third row)
  for (int j=0; j < 7; j++) { 
    // going by column
    Y[0][j] = x[1][j]; // dx/dt = v
    Y[1][j] = x[2][j]; // dv/dt = a
    Y[2][j] = 0.0; // da/dt = j: assumed zero
  } // end for loop

} // end f_tf

