#ifndef INCLUDE_CONTROLLERS_ACC_LIMITS_HPP
#define INCLUDE_CONTROLLERS_ACC_LIMITS_HPP

#include <iostream>

#include "eigen3/Eigen/Core"

int GetAccelerationLimitsFromPositionLimits(
    const double q, const double v,
    const double ql, const double qu,
    double *al, double *au,
    const int n, const double dt);

int GetAccelerationLimitsFromViability(
    const double q, const double v,
    const double ql, const double qu,
    const double amax,
    double *al, double *au,
    const int nq, const int nv, const double dt);

int GetAccelerationLimits(
    const Eigen::VectorXd &q, const Eigen::VectorXd &v,
    const Eigen::VectorXd &ql, const Eigen::VectorXd &qu,
    const Eigen::VectorXd &vmax, const Eigen::VectorXd &amax,
    Eigen::VectorXd &al, Eigen::VectorXd &au,
    const int nq, const int nv, const double dt);

#endif /* INCLUDE_CONTROLLERS_ACC_LIMITS_HPP */
