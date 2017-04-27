#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
}

void PID::UpdateError(double cte) {

  if(!init_d){
    d_error = cte;
    init_d = true;
    prev_cte = cte;
  }else{
    d_error = cte - prev_cte;
    prev_cte = cte;
  }
  p_error = cte;
  i_error +=cte;
}

void PID::UpdateSpeedError(double set_s, double desired) {
  p_error = set_s - desired;
  if(!init_d){
    d_error = p_error;
    init_d = true;
    prev_cte = p_error;
  }else{
    d_error = p_error - prev_cte;
    prev_cte = p_error;
  }

  i_error +=p_error;
}

double PID::TotalError() {
  return -Kp*p_error - Ki*i_error - Kd*d_error;


}
