#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double total_error;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  double prev_cte;
  double sum_cte;
  bool first;
  
  double coeff_delta[3];
  double best_err;
  int initial_settelment; 
  int round;
  int param_index;
  int add_sub;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void SetCoeffId(int coeff_id,double value);
  void AddCoeffId(int coeff_id,double value);


  double TestCoeff(double cte, int coeff_id, double value);
  void   SetCoeffAll(double Kp, double Ki, double Kd);

};

#endif /* PID_H */
