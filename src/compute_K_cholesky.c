#include "compute_K_cholesky.h"

// solves for the Kalman 

void forward_solve_L_3x3(const double L[3][3],
                         const double b[3],
                         double y[3])
{
    for (int i = 0; i < 3; i++) {
        double sum = b[i];
        for (int j = 0; j < i; j++) {
            sum -= L[i][j] * y[j];
        }
        y[i] = sum / L[i][i];
    }
}

void back_solve_LT_3x3(const double L[3][3],
                       const double y[3],
                       double x[3])
{
    for (int i = 2; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < 3; j++) {
            sum -= L[j][i] * x[j];  // NOTE: transpose access
        }
        x[i] = sum / L[i][i];
    }
}


void back_solve_L_3x3(const double L[3][3],
                     const double y[3],
                     double x[3])
{
    for (int i = 2; i >= 0; i--) {
        double sum = y[i];
        for (int j = i + 1; j < 3; j++) {
            sum -= L[i][j] * x[j];
        }
        x[i] = sum / L[i][i];
    }
}

void compute_K_cholesky(const double Pxz[3][3],
                        const double L[3][3],
                        double K[3][3])
{
    double b[3], y[3], x[3];

    for (int i = 0; i < 3; i++) {

        // b = row i of Pxz
        b[0] = Pxz[i][0];
        b[1] = Pxz[i][1];
        b[2] = Pxz[i][2];

        // L y = b
        forward_solve_L_3x3(L, b, y);

        // Lᵀ x = y
        back_solve_LT_3x3(L, y, x);

        // Store row of K
        K[i][0] = x[0];
        K[i][1] = x[1];
        K[i][2] = x[2];
    }
}




