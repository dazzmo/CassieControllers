#include "controllers/osc/tasks/joint_limits_task.h"

using namespace controller::osc;

JointLimitsTask::JointLimitsTask(const std::string &name, const DynamicModel::Size &sz) : Task("joint limits", 2 * sz.nq, sz) {
    beta_ = 0.5;
    zeta_ = Vector::Zero(sz.nq);
    ql_ = ConfigurationVector::Zero(sz.nq);
    qu_ = ConfigurationVector::Zero(sz.nq);
}

/**
 * @brief Function that activates (i.e. goes to a value of 1) as a joint approaches
 *  either of its limits. Transition rate is adjusted through beta.
 *
 * @param q
 * @param ql
 * @param qu
 * @return double
 */
double JointLimitsTask::TransitionFunction(double q, double ql, double qu) {
    double zeta = 1.0;
    double qu_tilde = qu - beta_;
    double ql_tilde = ql + beta_;

    if (q >= qu) {
        zeta = 1.0;
    } else if (qu_tilde < q && q < qu) {
        zeta = 0.5 + 0.5 * sin(M_PI * (q - qu_tilde) / beta_ - M_PI_2);
    } else if (ql_tilde <= q && q <= qu_tilde) {
        zeta = 0.0;
    } else if (ql < q && q < ql_tilde) {
        zeta = 0.5 + 0.5 * sin(M_PI * (q - ql_tilde) / beta_ - M_PI_2);
    } else {
        zeta = 1.0;
    }

    return zeta;
}

void JointLimitsTask::UpdateTask(const Vector &q, const Vector &v) {
    // Joint Limit and Velocity
    for (int i = 0; i < nq_; ++i) {
        zeta_[i] = TransitionFunction(q[i], ql_[i], qu_[i]);
    }
    // Task for keeping from boundaries
    x_.topRows(nq_) = (qu_ - q).cwiseProduct(zeta_);
    x_.bottomRows(nq_) = (q - ql_).cwiseProduct(zeta_);

    // Jacobian
    for (int i = 0; i < nq_; ++i) {
        J_(i, i) = -zeta_[i];
        J_(nq_ + i, i) = zeta_[i];
        dJdq_[i] = 0.0;
        dJdq_[nq_ + i] = 0.0;
    }

    // Compute task velocity
    dx_.topRows(nq_) = -v.cwiseProduct(zeta_);
    dx_.bottomRows(nq_) = v.cwiseProduct(zeta_);

    // Compute error
    e_ = x_;
    de_ = dx_;

    // Compute output
    pd_out_.topRows(nq_) = Kp().asDiagonal() * e_.topRows(nq_) + Kd().asDiagonal() * de_.topRows(nq_);
    pd_out_.bottomRows(nq_) = Kp().asDiagonal() * e_.bottomRows(nq_) + Kd().asDiagonal() * de_.bottomRows(nq_);
}
