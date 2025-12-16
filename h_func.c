#include <stdio.h>



double** h_func(double** H, double** x){

  double* h_prod[3][1];
  double tot_row;

  // H is a 3x3 matrix
  // x is a 3x1 matrix
  for (row=0; row < 3; row++){
    for (col=0; col < 3; col++){
      
      // find sum product for each row and add to row index of product matrix
      tot_row = H[row][col] * x[row][0] + H[row][col+1] * x[row+1][0] + H[row][col+2] * x[row+2][0]

    } // end inner for (col)
    
    h_prod[row][0] = tot_row
  } // end outer for (row)
  
  return h_prod;

} // end h_func
