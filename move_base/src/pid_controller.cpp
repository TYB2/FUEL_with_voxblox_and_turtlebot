#include "pid_controller.h"

using namespace std;
PIDcontroller::PIDcontroller( double dt, double max, double min, double Kp, double Ki, double Kd ) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Ki(Ki),
        _Kd(Kd),
        _prev_error(0),
        _integral(0)
{
}
PIDcontroller::PIDcontroller(){}
PIDcontroller::~PIDcontroller(){}

void PIDcontroller::setPIDParameters( double dt, double max, double min, double Kp, double Ki, double Kd ){
        _dt = dt;
        _max = max;
        _min = min;
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _prev_error = 0;
        _integral = 0;
}
double PIDcontroller::calculateU( double setpoint, double pv )
{
    //error
    double error = setpoint - pv;

    _integral += error * _dt;
    double derivative = (error - _prev_error) / _dt;

    // Proportional portion 
    //double Pout = _Kp * error;

    // Integral portion
    // double Iout = _Ki * _integral;

    // Derivative portion
    // double Dout = _Kd * derivative;

    // Total output
    double output = _Kp * error + _Ki * _integral + _Kd * derivative;

    // Limit to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _prev_error = error;

    return output;
}
