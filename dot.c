

void dot(MerweSigmaPoints *sp, double sigmas[7][3], double x_new[3]) {

  double sum = 0.0;
  
  for (int j=0; j < 3; j++){
    for (int i=0; i< 7; i++){
      sum = sum + (sp.Wm[j] * sigmas[i][j]);
      sum = sum + (sp.Wm[j] * sigmas[i][j]);
      sum = sum + (sp.Wm[j] * sigmas[i][j]);
    } // end nested for loop
    x_new[j] = sum;
    sum = 0.0;
  } // end for loop

} // end dot 
