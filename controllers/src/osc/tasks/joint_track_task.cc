#include "controllers/osc/tasks/joint_track_task.h"

using namespace controller::osc;

JointTrackTask::JointTrackTask(const DynamicModel::Size &sz) : Task("joint track", sz.nq, sz) {
    nq_ = sz.nq;
    nv_ = sz.nv;
}

void JointTrackTask::Update(const Vector &q, const Vector &v) {
    // Task for tracking joint trajectory
    x_ = q;

    // Jacobian
    for (int i = 0; i < nv_; ++i) {
        J_(i, i) = 1.0;
        dJdt_v_[i] = 0.0;
    }

    // Compute task velocity
    dx_ = v;

    // Compute error
    e_ = x_ - r_;
    de_ = dx_ - dr_;

    // Output
    pd_out_ = -(Kp() * e_ + Kd() * de_);
}
