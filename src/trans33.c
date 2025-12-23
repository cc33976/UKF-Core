#include "trans33.h"

void trans33(double A[3][3], double At[3][3]) {
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++) {
      At[j][i] = A[i][j]; 
    } // end nested for loop
  } // end outer for loop 

} // end trans 
