#include "osc_fixed.h"

/**
 * @brief Remaps the measured state to the state expected by our model
 *
 * @param nq
 * @param q
 * @param nv
 * @param v
 */
void CassieFixedOSC::UpdateState(int nq, const double *q, int nv,
                                 const double *v) {
    // Map Mujoco data to appropriate joints
    qpos_ << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13], q[14], q[15],
        q[16], q[21], q[22], q[23], q[24], q[27];
    qvel_ << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12], v[13], v[14],
        v[15], v[19], v[20], v[21], v[22], v[25];

    program_.SetParameters("qpos", qpos_);
    program_.SetParameters("qvel", qvel_);
}

void CassieFixedOSC::UpdateReferences(double time) {
    double l_phase = -(2.0 * M_PI / 4.0) * time;

    // Update all end-effector state estimates
    for (osc::EndEffector &e : ee_) {
        e.Function().setInput(1, qpos_.data());
        e.Function().setInput(2, qvel_.data());
        e.Function().call();
    }

    // Create tracking references
    Eigen::Vector3d &xl = tracking_tasks_["foot_l"].xr,
                    &xr = tracking_tasks_["foot_r"].xr;

    std::cout << "Left: "
              << ee_[ee_idx_[tracking_tasks_["foot_l"].end_effector_id]]
                     .EvalPosition()
                     .transpose()
              << std::endl;
    std::cout << "Right: "
              << ee_[ee_idx_[tracking_tasks_["foot_r"].end_effector_id]]
                     .EvalPosition()
                     .transpose()
              << std::endl;

    // Create trajectories
    xl[0] = 0.0 + 0.2 * cos(l_phase);
    xl[1] = 0.1;
    xl[2] = -0.7 + 0.2 * sin(l_phase);

    xr[0] = 0.0 + 0.2 * cos(l_phase);
    xr[1] = -0.1;
    xr[2] = -0.7 + 0.2 * sin(l_phase);

    tracking_tasks_["foot_l"].qr = osc::RPYToQuaterion(0, 0, 0);
    tracking_tasks_["foot_r"].qr = osc::RPYToQuaterion(0, 0, 0);

    // Compute new tracking errors
    Eigen::VectorXd xaccd = osc::DesiredTrackingTaskAcceleration(
        tracking_tasks_["foot_l"],
        ee_[tracking_tasks_["foot_l"].end_effector_id]);
    program_.SetParameters("foot_l_xaccd", -xaccd);

    std::cout << xaccd.transpose() << std::endl;

    xaccd = osc::DesiredTrackingTaskAcceleration(
        tracking_tasks_["foot_r"],
        ee_[tracking_tasks_["foot_r"].end_effector_id]);
    program_.SetParameters("foot_r_xaccd", -xaccd);

    std::cout << xaccd.transpose() << std::endl;
}