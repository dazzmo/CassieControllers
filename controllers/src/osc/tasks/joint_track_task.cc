#include "controllers/osc/tasks/joint_track_task.h"

using namespace controller::osc;

JointTrackTask::JointTrackTask(const DynamicModel::Size &sz) : Task("joint track", sz.nq, sz) {
}

void JointTrackTask::UpdateTask(const Vector &q, const Vector &v) {
    // Task for tracking joint trajectory
    x_ = q;

    // Jacobian
    for (int i = 0; i < nv_; ++i) {
        J_(i, i) = 1.0;
        dJdq_[i] = 0.0;
    }


    // Compute task velocity
    dx_ = v;

    // Compute error
    e_ = x_ - r_;
    de_ = dx_ - dr_;

    // Output
    pd_out_ = Kp().asDiagonal() * e_ + Kd().asDiagonal() * de_;
}
