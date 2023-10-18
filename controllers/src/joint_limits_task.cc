#include "controllers/joint_limits_task.h"

JointLimitsTask::JointLimitsTask(int nq, int nv) : Task(2 * nq, nv, nullptr) {
    qpos_l_ = Eigen::VectorXd::Zero(nq);
    qpos_u_ = Eigen::VectorXd::Zero(nq);

    Kp.resize(nq);
    Kd.resize(nv);
}

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

int JointLimitsTask::UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true) {
    // Joint Limit and Velocity
    for (int i = 0; i < nq_; i++) {
        zeta_[i] = TransitionFunction(qpos[i], qpos_l_[i], qpos_u_[i]);
    }
    x_.bottomRows(nq_) = Kp * (qpos - qpos_l_) - Kd * qvel;
    x_.topRows(nq_) = Kp * (qpos_u_ - qpos) - Kd * qvel;

    // Jacobian
    if (update_jacobians) {
        // Assess if near joint bound
        for (int i = 0; i < nq_; i++) {
            if(qpos_u_[i] - qpos[i] > beta_) {
                J_(i, i) = 1.0;
            } else if(qpos[i] - qpos_l_[i] < beta_) {
                J_(nq_ + i, i) = 1.0;
            } else {
                J_(i, i) = 0.0;
                J_(nq_ + i, i) = 0.0;
            }
        }
    }

    // Compute task velocity
    dx_ = J_ * qvel;

    return 0;
}
