#include "controllers/tasks/joint_track_task.h"

JointTrackTask::JointTrackTask(int nq, int nv) : Task(nq, nv, "joint track", nullptr) {
}

int JointTrackTask::UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians) {
    // Task for tracking joint trajectory
    x_ = qpos;

    // Jacobian
    if (update_jacobians) {
        // Assess if near joint bound
        for (int i = 0; i < nv_; i++) {
            J_(i, i) = 1.0;
        }
    }

    // Compute task velocity
    dx_ = qvel;

    return 0;
}
