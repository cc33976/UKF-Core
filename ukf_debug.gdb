# =========================
# UKF Debugging Script
# =========================

# Stop at main first
break main

# Optional: break at key UKF functions
break compute_process_sigmas
break unscented_transform
break update

# Automatically print UKF state and covariance when hitting breakpoints
define print_ukf
    echo ======== UKF STATE ========\n
    printf "x = [%.6f, %.6f, %.6f]\n", ukf->x[0], ukf->x[1], ukf->x[2]
    echo ---------\n
    echo P = \n
    printf "%.6f %.6f %.6f\n", ukf->P[0][0], ukf->P[0][1], ukf->P[0][2]
    printf "%.6f %.6f %.6f\n", ukf->P[1][0], ukf->P[1][1], ukf->P[1][2]
    printf "%.6f %.6f %.6f\n", ukf->P[2][0], ukf->P[2][1], ukf->P[2][2]
    echo ---------\n
    echo Sigmas = \n
    set $i = 0
    while $i < 7
        printf "%.6f %.6f %.6f\n", ukf->sigmas_f[$i][0], ukf->sigmas_f[$i][1], ukf->sigmas_f[$i][2]
        set $i = $i + 1
    end
    echo ===========================\n
end

# Whenever we hit a breakpoint, run this
commands
    print_ukf
    continue
end

# Start running the program
run
