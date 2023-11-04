#ifndef INCLUDE_CONTROLLERS_JOINT_TRACK_TASK_HPP
#define INCLUDE_CONTROLLERS_JOINT_TRACK_TASK_HPP

#include "controllers/osc/tasks/task.h"

class JointTrackTask : public Task {
   public:
    JointTrackTask(int nq, int nv);
    int UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true);

   private:
};

#endif /* INCLUDE_CONTROLLERS_JOINT_TRACK_TASK_HPP */
