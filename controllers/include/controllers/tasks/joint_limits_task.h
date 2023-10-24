#ifndef INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP
#define INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP
/**
 * @file joint_limits_task.h
 * @author dazzmo
 * @brief Task created to avoid joint limits, with soft buffers to encourage accelerations which stray joints from the
 * hard limits rather than acting only when the joints are almost at their limits
 * @version 0.1
 * @date 18-10-2023
 *
 */

#include "controllers/tasks/task.h"

class JointLimitsTask : public Task {
   public:
    JointLimitsTask(int nq, int nv);

    void SetUpperPositionLimit(const Eigen::VectorXd &qpos_u) { qpos_u_ = qpos_u; }
    void SetLowerPositionLimit(const Eigen::VectorXd &qpos_l) { qpos_l_ = qpos_l; }

    void SetProportionalErrorGain(const Eigen::VectorXd &Kp) {
        LOG(INFO) << "Kp: " << Kp_.transpose();
        Kp_.topRows(nq_) << Kp;
        Kp_.bottomRows(nq_) << Kp;
    }

    void SetDerivativeErrorGain(const Eigen::VectorXd &Kd) {
        Kd_.topRows(nq_) << Kd;
        Kd_.bottomRows(nq_) << Kd;
    }

    int UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true);

   private:
    double beta_ = 1.0;
    Eigen::VectorXd zeta_;
    Eigen::VectorXd qpos_l_;
    Eigen::VectorXd qpos_u_;

    double TransitionFunction(double q, double ql, double qu);
};

#endif /* INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP */
