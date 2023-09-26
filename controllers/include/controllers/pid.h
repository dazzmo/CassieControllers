#ifndef INCLUDE_CONTROLLERS_PID_HPP
#define INCLUDE_CONTROLLERS_PID_HPP

#include <iostream>

struct PGains {
    double kp = 0.0;
};

struct PDGains {
    double kp = 0.0;
    double kd = 0.0;
};

struct PIDGains {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
};

struct PIGains {
    double kp = 0.0;
    double ki = 0.0;
}


#endif /* INCLUDE_CONTROLLERS_PID_HPP */
