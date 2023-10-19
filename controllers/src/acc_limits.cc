#include "controllers/tasks/acc_limits.h"

int GetAccelerationLimitsFromPositionLimits(
    const double q, const double v,
    const double ql, const double qu,
    double *al, double *au,
    const int n, const double dt) {
    std::cout << "GetAccelerationLimitsFromPositionLimits" << '\n';

    double aM1 = -(1.0 / dt) * v;
    double aM2 = -0.5 * pow(v, 2) / (qu - q);
    double aM3 = 2.0 * (qu - q - dt * v) / pow(dt, 2);
    double am2 = 0.5 * pow(v, 2) / (q - ql);
    double am3 = 2.0 * (ql - q - dt * v) / pow(dt, 2);

    if (v >= 0) {
        *al = am3;
        if (aM3 > aM1) {
            *au = aM3;
        } else {
            *au = std::min(aM1, aM2);
        }
    } else {
        *au = aM3;
        if (am3 < aM1) {
            *al = am3;
        } else {
            *al = std::max(aM1, am2);
        }
    }

    return 0;
}

int GetAccelerationLimitsFromViability(
    const double q, const double v,
    const double ql, const double qu,
    const double amax,
    double *al, double *au,
    const int nq, const int nv, const double dt) {

    double a = pow(dt, 2);
    double b = dt * (2.0 * v + dt * amax);
    double c = pow(v, 2) - 2.0 * amax * (qu - q - dt * v);

    double d = 2.0 * dt * v - pow(dt, 2) * amax;
    double e = pow(v, 2) - 2.0 * amax * (q + dt * v - ql);

    double a1 = -(1.0 / dt) * v;

    double discr1 = pow(b, 2) - 4 * a * c;
    double discr2 = pow(d, 2) - 4 * a * e;

    if (discr1 >= 0) {
        *au = std::max(a1, (-b + sqrt(discr1)) / (2.0 * a));
    } else {
        *au = a1;
    }

    if (discr2 >= 0) {
        *al = std::min(a1, (-b - sqrt(discr2)) / (2.0 * a));
    } else {
        *al = a1;
    }

    return 0;
}

int GetAccelerationLimits(
    const Eigen::VectorXd &q, const Eigen::VectorXd &v,
    const Eigen::VectorXd &ql, const Eigen::VectorXd &qu,
    const Eigen::VectorXd &vmax, const Eigen::VectorXd &amax,
    Eigen::VectorXd &al, Eigen::VectorXd &au,
    const int nq, const int nv, const double dt) {
    double qacc_l[4], qacc_u[4];

    for (int i = 0; i < nv; ++i) {
        GetAccelerationLimitsFromPositionLimits(q[i], v[i], ql[i], qu[i],
                                                qacc_l, qacc_u,
                                                nv, dt);
        qacc_l[1] = (-vmax[i] - v[i]) / dt;
        qacc_u[1] = (vmax[i] - v[i]) / dt;

        GetAccelerationLimitsFromViability(q[i], v[i], ql[i], qu[i], amax[i],
                                           qacc_l + 2, qacc_u + 2,
                                           nq, nv, dt);

        qacc_l[3] = -amax[i];
        qacc_u[3] = amax[i];

        for(int i = 0; i < 4; ++i) std::cout << qacc_l[i] << '\t';
        std::cout << '\n';
        for(int i = 0; i < 4; ++i) std::cout << qacc_u[i] << '\t';
        std::cout << '\n';

        al[i] = std::max(qacc_l[0], std::max(qacc_l[1], std::max(qacc_l[2], qacc_l[3])));
        au[i] = std::min(qacc_u[0], std::min(qacc_u[1], std::min(qacc_u[2], qacc_u[3])));
    }

    return 0;
}