#include "controllers/osc/tasks/end_effector_task.h"

using namespace controller::osc;

EndEffectorTask::EndEffectorTask(
    const std::string& name, const DynamicModel::Size& sz,
    Task::TaskCallbackFunction& callback,
    const EndEffectorTask::TrackingType tracking_type)
    : Task(name, 6, sz, callback) {
    inContact = false;
    mu_ = 1.0;  // TODO: Make this a default setting
    lambda_.setZero();
    normal_ = Eigen::Vector3d::UnitZ();

    // Resize the task reference configuration vector to handle the tracking
    // task
    if (tracking_type == TrackingType::kRotation) {
        Task::Resize(sz, 3);
        // 3D vector reference
        r_.resize(3);
    } else if (tracking_type == TrackingType::kRotation) {
        Task::Resize(sz, 3);
        // Quaternion reference
        r_.resize(4);
    } else if (tracking_type == TrackingType::kTranslationAndRotation) {
        Task::Resize(sz, 6);
        // 3D vector and quaternion reference
        r_.resize(7);
    }
}

void EndEffectorTask::Update(const ConfigurationVector& q,
                             const TangentVector& v) {
    // Perform callback
    if (callback_ != nullptr) {
        callback_(q, v, x_, J_, dJdt_v_);
    } else {
        throw std::runtime_error("Task callback for " + name_ + " is null!");
    }

    // Update velocity
    dx_ = J_ * v;

    // Compute error in translational and rotational error
    if (track_type_ == TrackingType::kTranslation) {
        // Euclidean distance metric error
        e_ = x_ - r_;
        de_ = dx_ - dr_;
    } else if (track_type_ == TrackingType::kRotation) {
        // Given the quaternion of the orientation of the end-effector,
        // determine the rotation matrix for the system
        Eigen::Quaternion<double> qR = x, qD = r();
        Eigen::Matrix3d R = qR.toRotationMatrix();
        // Convert reference quaternion to rotation matrix
        Eigen::Matrix3d Rd = qD.toRotationMatrix();
        // Determine rotation error matrix \f$ e = R^T R_d \f$
        Eigen::Matrix3d Re = R.transpose() * Rd;
        // Logarithm map
        double theta = acos((Re.trace() - 1.0) / 2.0);
        Eigen::Matrix3d logR =
            (theta / (2.0 * sin(theta))) * (Re - Re.transpose());
        // Skew-symmetric matrix logR can then be converted to Lie algebra by
        // extracting the xyz coordinates
        Eigen::Vector3d omega(logR(2, 1), -logR(2, 0), logR(1, 0));

        e_ = omega;
        // Velocity error is simple Euclidean metric
        de_ = dx_ - dr_;

    } else if (track_type_ == TrackingType::kTranslationAndRotation) {
    }

    // Compute PD error
    pd_out_ = -(Kp_ * e_ + Kd_ * de_);
}