#include "PID.h"
#include <algorithm>
#include <limits>

PID::PID(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  prev_cte = 0.0;

  counter = 0; 
  errorSum = 0.0;

  // Use std::numeric_limits for min/max initialization
  minError = std::numeric_limits<double>::max();
  maxError = std::numeric_limits<double>::lowest();
}

PID::~PID() {}

void PID::UpdateError(double cte) {
  // Proportional Error (P)
  p_error = cte;

  // Integral Error (I)
  i_error += cte;

  // Differential Error (D)
  // Initially prev_cte is 0, so d_error is just cte.
  // This is acceptable for the first step.
  d_error = cte - prev_cte;

  prev_cte = cte;

  errorSum += cte;
  counter++;

  if (cte > maxError) {
    maxError = cte;
  }
  if (cte < minError) {
    minError = cte;
  }
}

double PID::TotalError() {
  // Total Error = Kp * p_error + Ki * i_error + Kd * d_error
  // Note: Usually the control output is -TotalError
  return p_error * Kp + i_error * Ki + d_error * Kd;
}

double PID::AverageError() {
  if (counter == 0) return 0.0;
  return errorSum / counter;
}

double PID::MinError() {
  return minError;
}

double PID::MaxError() {
  return maxError;
}
