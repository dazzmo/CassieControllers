#ifndef INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP
#define INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP

#include "controllers/task.h"

class JointLimitsTask : public Task {
   public:
    Eigen::DiagonalMatrix<double, -1> Kp;
    Eigen::DiagonalMatrix<double, -1> Kd;
    
    JointLimitsTask(int nq, int nv);
    
    int UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true);

   private:
    int nq_;
    int nv_;
    double beta_ = 1.0;
    Eigen::VectorXd zeta_;
    Eigen::VectorXd qpos_l_;
    Eigen::VectorXd qpos_u_;
    
    double TransitionFunction(double q, double ql, double qu);
    
};

#endif /* INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP */
