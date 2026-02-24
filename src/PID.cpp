#include "PID.h"
#include <algorithm>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  prev_cte = 0.0; // Previous CTE

  counter = 0; 
  errorSum = 0.0;
  minError = std::numeric_limits<double>::max();
  maxError = std::numeric_limits<double>::min();

}

void PID::UpdateError(double cte) {
  p_error = cte; // Proportional Error (P)

  i_error += cte; // Integral Error (I)

  d_error = cte - prev_cte; // Diferential Error (D)

  prev_cte = cte;

  errorSum += cte;
  counter++;

  if ( cte > maxError ) {
    maxError = cte;
  }
  if ( cte < minError ) {
    minError = cte;
  }
}

double PID::TotalError() {
  return p_error * Kp + i_error * Ki + d_error * Kd;
}

// Calculate Average/Min/Max Error
double PID::AverageError() {
  return errorSum/counter;
}

double PID::MinError() {
  return minError;
}

double PID::MaxError() {
  return maxError;
}