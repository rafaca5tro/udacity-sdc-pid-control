#ifndef PID_H
#define PID_H

class PID {
 public:
  /**
   * Constructor
   * @param Kp_ The initial proportional coefficient
   * @param Ki_ The initial integral coefficient
   * @param Kd_ The initial differential coefficient
   */
  PID(double Kp_, double Ki_, double Kd_);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @return The total PID error
   */
  double TotalError();

  /**
   * Returns the average error.
   * @return The average error
   */
  double AverageError();

  /**
   * Returns the min error.
   * @return The minimum error encountered
   */
  double MinError();

  /**
   * Returns the max error.
   * @return The maximum error encountered
   */
  double MaxError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double prev_cte;

  /**
   * Error counters
   */
  long counter;
  double errorSum;
  double minError;
  double maxError;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;
};

#endif  // PID_H
