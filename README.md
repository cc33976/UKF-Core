# Unscented Kalman Filter (UKF) in C

A from-scratch implementation of an Unscented Kalman Filter (UKF) in **C**, designed for nonlinear state estimation in embedded / aerospace applications.

This project was developed as part of a rocket state estimation effort and includes:
- Sigma point generation (Merwe scaled sigma points)
- RK4-based nonlinear state propagation
- Joseph-stabilized covariance update
- Cholesky-based Kalman gain computation
- Full predict/update loop with tunable noise models

No external math libraries. No magic. Just linear algebra, pain, and eventually convergence.

---

## State Definition

The current implementation estimates a 3-state system:

```text
x = [ altitude, velocity, acceleration ]
# UnscentedKalmanFilter
Unscented Kalman Filter written in C for the purpose of improved data resolution in HPR telemetry


## File Structure
.
├── src/
│   └── *.c          # All source files
├── include/
│   └── *.h          # All header files
├── Makefile
├── README.md
└── run.sh

## Build and Run

The project is built and executed using a simple shell script.

```bash
chmod +x run.sh
./run.sh

## temporary block diagram
Initial State Guess
(x₀, P₀)
      │
      ▼
Sigma Point Generation
(Merwe Scaled Sigma Points)
      │
      ▼
Nonlinear Propagation
(RK4 on each sigma point)
      │
      ▼
Propagated Sigma Points
(sigmas_f)
      │
      ▼
Unscented Transform
      │
      ├──► Predicted State (x⁻)
      └──► Predicted Covariance (P⁻)
      │
      ▼
──────────── UPDATE ────────────
      │
      ▼
Measurement Sigma Points
(h(sigmas_f))
      │
      ▼
Unscented Transform
      │
      ├──► Predicted Measurement (ẑ)
      └──► Innovation Covariance (S)
      │
      ▼
Cross Covariance (Pₓz)
      │
      ▼
Kalman Gain (K)
      │
      ▼
State Update
x⁺ = x⁻ + K (z − ẑ)
      │
      ▼
Covariance Update
(Joseph Form)
      │
      ▼
Posterior State
(x⁺, P⁺)
      │
      └──► Used as prior next iteration








