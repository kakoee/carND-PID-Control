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

    coeff_delta[0]= 0.5*Kp;
    coeff_delta[1] = 0.5*Ki;
    coeff_delta[2] = 0.5*Kd;   
    best_err=99999;
    initial_settelment = 1;
    round=0; 
    param_index=0;
    add_sub=0;

}

void PID::UpdateError(double cte) {
    
    sum_cte +=cte;
    if(first)
            prev_cte=cte;
    double diff_cte = cte - prev_cte;
    
    p_error=  cte;
    i_error = sum_cte ;
    d_error = diff_cte ;
    
    prev_cte=cte;
    first=false;
    

    if(round<=initial_settelment){
        round++;
    }
    else{
        if(round%100!=0)
            total_error=total_error+cte*cte;
        else{
            if(add_sub==0){
                AddCoeffId(param_index,coeff_delta[param_index]);
                add_sub=1;
            }
            else if(add_sub==1){
                if(total_error<best_err){
                    best_err=total_error;
                    coeff_delta[param_index]*=1.1;
                    param_index++;
                    param_index=param_index%3;
                    add_sub=0;
                }
                else{
                    AddCoeffId(param_index,-2*coeff_delta[param_index]);
                    add_sub=2;
                }
            }
            else if(add_sub==2){
                if(total_error<best_err){
                    best_err=total_error;
                    coeff_delta[param_index]*=1.1;
                    param_index++;
                    param_index=param_index%3;
                    add_sub=0;
                }
                else{
                    AddCoeffId(param_index,coeff_delta[param_index]);
                    coeff_delta[param_index]*=0.9;
                    add_sub=0;
                }

            }
            total_error=0;
        }

        round++;
    }

if(round>10000)
    round=initial_settelment+1;


}

double PID::TotalError() {
    
    return - Kp * p_error 
           - Kd * d_error 
           - Ki * i_error;

}

void PID::SetCoeffId(int coeff_id,double value){

    switch(coeff_id){
        case 0:
            Kp=value;
            break;
        case 1:
            Ki=value;
            break;
        case 2:
            Kd=value;
            break;
        default:
            Kp=value;
            break;
    }

}

void PID::AddCoeffId(int coeff_id,double value){

    switch(coeff_id){
        case 0:
            Kp+=value;
            break;
        case 1:
            Ki+=value;
            break;
        case 2:
            Kd+=value;
            break;
        default:
            Kp+=value;
            break;
    }

}



double PID::TestCoeff(double cte,int coeff_id,double value){
    double sum_cte1 =sum_cte+cte;
    double prev_cte1;
    if(first)
            prev_cte1=cte;
    else 
            prev_cte1=prev_cte;
    double diff_cte = cte - prev_cte1;
    double Kp1,Ki1,Kd1;
    switch(coeff_id){
        case 0:
            Kp1=value;
            break;
        case 1:
            Ki1=value;
            break;
        case 2:
            Kd1=value;
            break;
        default:
            Kp1=value;
            break;
    }    

    double p_error1= -Kp1 * cte;
    double i_error1 = -sum_cte1 * Ki1;
    double d_error1 = -diff_cte * Kd1;
    
    double total_error1 = p_error1 + i_error1 + d_error1;
    return total_error1;

}

void PID::SetCoeffAll(double Kp, double Ki, double Kd) {

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

}

