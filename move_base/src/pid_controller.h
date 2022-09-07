#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <iostream>
#include <cmath>

class PIDcontroller
{
public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PIDcontroller();
    PIDcontroller( double dt, double max, double min, double Kp, double Ki, double Kd );
    void setPIDParameters( double dt, double max, double min, double Kp, double Ki, double Kd );
    // Returns the manipulated variable given a setpoint and current process value
    double calculateU( double setpoint, double pv );
    ~PIDcontroller();

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _prev_error;
    double _integral;
};
#endif PID_CONTROLLER_H
