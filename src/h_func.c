#include "h_func.h"
#include "UnscentedKalmanFilter.h"



void h_func(double x_in[3], double H[3][3], double z_out[3]){

  double tot_row; 

  // H is a 3x3 matrix
  // x is a 3x1 matrix
  for (int i=0; i < 3; i++){
    tot_row = 0.0;
    for (int j=0; j < 3; j++){
      
      // find sum product for each row and add to row index of product matrix
      tot_row += H[i][j] * x_in[j];

    } // end inner for (col)
    
    z_out[i] = tot_row;
  } // end outer for (row)

} // end h_func
