#include "dot33.h"

// computes the dot product of two 3x3 matrices
void dot33(double A[3][3], double B[3][3], double Out[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double sum = 0.0;
            for (int k = 0; k < 3; k++) {
                sum += A[i][k] * B[k][j];
            } // end nested for loop
            Out[i][j] = sum;
        } // end middle for loop
    } // end outermost for loop
} // end dot33

