

void residual(double sigmas[3], double x[3], double y[3]) {

  for (int i=0; i < 3; i++){
    y[i] = x[i] - sigmas[i];
  } // end for loop 
  

} // end residual 
