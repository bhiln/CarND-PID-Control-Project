#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;
  PID::err = 0;
  PID::n = 1;
  PID::itter = 0;
}

void PID::UpdateError(double cte) {
  if (itter > n){
    err += cte*cte;
  }
}

double PID::TotalError() {
  return err/n;
}

