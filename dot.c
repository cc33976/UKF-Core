

// might be a good idea to make Wm and x_new ambiguous (a,b) 
void dot(double Wm[7], double sigmas[7][3], double x_new[3]) {

  double sum = 0.0;
  
  for (int j=0; j < 3; j++){
    for (int i=0; i< 7; i++){
      sum = sum + (Wm[i] * sigmas[i][j]);
      sum = sum + (Wm[i] * sigmas[i][j]);
      sum = sum + (Wm[i] * sigmas[i][j]);
    } // end nested for loop
    x_new[j] = sum;
    sum = 0.0;
  } // end for loop

} // end dot 
