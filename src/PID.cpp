#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->sum_cte =0;
    this->prev_cte=0;
    first = true;
    p_error=0;
    i_error=0;
    d_error=0;  
    total_error = 0;
}

void PID::UpdateError(double cte) {
    
    sum_cte +=cte;
    if(first)
            prev_cte=cte;
    double diff_cte = cte - prev_cte;
    
    p_error= -Kp * cte;
    i_error = -sum_cte * Ki;
    d_error = -diff_cte * Kd;
    
    total_error = p_error + i_error + d_error;
    
    prev_cte=cte;
    first=false;
}

double PID::TotalError() {
    
    return total_error;

}

