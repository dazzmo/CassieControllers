#include "controllers/osc/tasks/joint_limits_task.h"

JointLimitsTask::JointLimitsTask(int nq, int nv) : Task(2 * nq, nv, "joint limits") {
    nq_ = nq;
    beta_ = 0.5;
    zeta_ = Eigen::VectorXd::Zero(nq);
    qpos_l_ = Eigen::VectorXd::Zero(nq);
    qpos_u_ = Eigen::VectorXd::Zero(nq);
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

int JointLimitsTask::UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians) {
    
    // Joint Limit and Velocity
    for (int i = 0; i < nq_; i++) {
        zeta_[i] = TransitionFunction(qpos[i], qpos_l_[i], qpos_u_[i]);
    }
    // Task for keeping from boundaries
    x_.topRows(nq_) = (qpos_u_ - qpos).cwiseProduct(zeta_);
    x_.bottomRows(nq_) = (qpos - qpos_l_).cwiseProduct(zeta_);

    // Jacobian
    if (update_jacobians) {
        // Assess if near joint bound
        for (int i = 0; i < nq_; i++) {
            J_(i, i) = -zeta_[i];
            J_(nq_ + i, i) = zeta_[i];
        }
    }

    // Compute task velocity
    dx_.topRows(nq_) = -qvel.cwiseProduct(zeta_);
    dx_.bottomRows(nq_) = qvel.cwiseProduct(zeta_);

    return 0;
}
