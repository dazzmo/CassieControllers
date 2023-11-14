#include "controllers/osc/tasks/joint_limits_task.h"

using namespace controller;
using namespace controller::osc;

JointLimitsTask::JointLimitsTask(const DynamicModel::Size &sz) : Task("joint limits", 2 * sz.nq, sz) {
    beta_ = 0.5;

    nq_ = sz.nq;
    nv_ = sz.nv;

    zeta_ = Vector::Zero(sz.nq);
    qmin_ = ConfigurationVector::Zero(sz.nq);
    qmax_ = ConfigurationVector::Zero(sz.nq);
}

/**
 * @brief Function that activates (i.e. goes to a value of 1) as a joint approaches
 *  either of its limits. Transition rate is adjusted through beta.
 *
 * @param q
 * @param qmin
 * @param qmax
 * @return Scalar
 */
Scalar JointLimitsTask::TransitionFunction(Scalar q, Scalar qmin, Scalar qmax) {
    Scalar zeta = 1.0;
    Scalar qmax_tilde = qmax - beta_;
    Scalar qmin_tilde = qmin + beta_;

    if (q >= qmax) {
        zeta = 1.0;
    } else if (qmax_tilde < q && q < qmax) {
        zeta = 0.5 + 0.5 * sin(M_PI * (q - qmax_tilde) / beta_ - M_PI_2);
    } else if (qmin_tilde <= q && q <= qmax_tilde) {
        zeta = 0.0;
    } else if (qmin < q && q < qmin_tilde) {
        zeta = 0.5 + 0.5 * sin(M_PI * (q - qmin_tilde) / beta_ - M_PI_2);
    } else {
        zeta = 1.0;
    }

    return zeta;
}

void JointLimitsTask::Update(const Vector &q, const Vector &v) {
    // Joint Limit and Velocity
    for (int i = 0; i < nq_; ++i) {
        zeta_[i] = TransitionFunction(q[i], qmin_[i], qmax_[i]);
    }
    // Task for keeping from boundaries
    x_.topRows(nq_) = (qmax_ - q).cwiseProduct(zeta_);
    x_.bottomRows(nq_) = (q - qmin_).cwiseProduct(zeta_);

    // Jacobian
    for (int i = 0; i < nq_; ++i) {
        J_(i, i) = -zeta_[i];
        J_(nq_ + i, i) = zeta_[i];
        dJdt_v_[i] = 0.0;
        dJdt_v_[nq_ + i] = 0.0;
    }

    // Compute task velocity
    dx_.topRows(nq_) = -v.cwiseProduct(zeta_);
    dx_.bottomRows(nq_) = v.cwiseProduct(zeta_);

    // Compute error
    e_ = x_;
    de_ = dx_;

    // Compute output
    pd_out_.topRows(nq_) = Kp() * e_.topRows(nq_) + Kd() * de_.topRows(nq_);
    pd_out_.bottomRows(nq_) = Kp() * e_.bottomRows(nq_) + Kd() * de_.bottomRows(nq_);
}
