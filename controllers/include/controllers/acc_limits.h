#ifndef INCLUDE_CONTROLLERS_ACC_LIMITS_HPP
#define INCLUDE_CONTROLLERS_ACC_LIMITS_HPP

/**
 * @file acc_limits.h
 * @author dazzmo
 * @brief These task-based position and velocity limits are based on viability analysis, with our implementation
 * following the method in "Joint Position and Velocity Bounds in Discrete-Time Acceleration/Torque Control of
 * Robot Manipulators" by Andrea del Prete (https://ieeexplore.ieee.org/document/8007337).
 * @version 0.1
 * @date 18-10-2023
 *
 */

#include <iostream>

#include "eigen3/Eigen/Core"

int GetAccelerationLimitsFromPositionLimits(
    const double q, const double v,
    const double qmin, const double qmax,
    double *al, double *au,
    const int n, const double dt);

int GetAccelerationLimitsFromViability(
    const double q, const double v,
    const double qmin, const double qmax,
    const double amax,
    double *al, double *au,
    const int nq, const int nv, const double dt);

int GetAccelerationLimits(
    const Eigen::VectorXd &q, const Eigen::VectorXd &v,
    const Eigen::VectorXd &qmin, const Eigen::VectorXd &qmax,
    const Eigen::VectorXd &vmax, const Eigen::VectorXd &amax,
    Eigen::VectorXd &al, Eigen::VectorXd &au,
    const int nq, const int nv, const double dt);

#endif /* INCLUDE_CONTROLLERS_ACC_LIMITS_HPP */
