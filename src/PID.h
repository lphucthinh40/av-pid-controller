#ifndef PID_H
#define PID_H

enum TwiddleState {BEST_ERROR_INIT, TRY_ADDITION, TRY_SUBTRACTION, OFFICIAL_UPDATE};

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, bool enable_twiddle_= false, int n_ = 100);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  void UpdateTwiddle(double cte);
  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  bool reset_simulator;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double K[3];
  double dK[3];


  /**
   * Twiddle
   */
  bool better_found;
  bool enable_twiddle;
  int n;
  int n_warmup;
  int counter;
  int rotary_index;
  double best_error;
  double average_error;
  double sum_error;
  TwiddleState state;
};

#endif  // PID_H