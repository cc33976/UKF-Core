#include "residual.h"
#include <stdio.h>

void residual(double a[3], double b[3], double out[3]) {

  printf("#### entering residual ####\n");

  printf("a: ");
  for (int i=0;i<3;i++){
    printf("%.4f ",a[i]);
  } // end for
  printf("\n");

  printf("b: ");
  for (int i=0;i<3;i++){
    printf("%.4f ",b[i]);
  } // end for
  printf("\n");

  for (int i=0; i < 3; i++){
    out[i] = b[i] - a[i];
  } // end for loop

  printf("out: ");
  for (int i=0;i<3;i++){
    printf("%.4f ",out[i]);
  } // end for
  printf("\n");

  printf("#### leaving residual ####\n");

} // end residual 
