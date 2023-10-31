#include "osc.h"

void ArmOSC::SetupController() {
    LOG(INFO) << "InitMatrices";

    M_ = Eigen::MatrixXd::Zero(ARM_MODEL_NV, ARM_MODEL_NV);
    B_ = Eigen::MatrixXd::Zero(ARM_MODEL_NV, ARM_MODEL_NU);
    h_ = Eigen::VectorXd::Zero(ARM_MODEL_NV);

    qpos_0() << 0.0, 0.0, 0.0;
    qpos_bl() << -M_PI, -M_PI, -M_PI;
    qpos_bu() << M_PI, M_PI, M_PI;
    ctrl_max() << 15.0, 15.0, 15.0;
    qvel_max().setConstant(1e1);
    qacc_max().setConstant(1e20);

    RegisterEndEffectorTask("tip", arm_tip);
    GetEndEffectorTaskMap()["tip"]->SetTaskWeighting(1e1);
    Eigen::VectorXd Kp(3), Kd(3);
    Kp << 1e2, 1e2, 1e2;
    Kd << 1e1, 1e1, 1e1;
    GetEndEffectorTaskMap()["tip"]->SetProportionalErrorGain(Kp);
    GetEndEffectorTaskMap()["tip"]->SetDerivativeErrorGain(Kd);
    
    Eigen::VectorXd Kp_jt(3), Kd_jt(3);
    Kp_jt << 0.0, 0.0, 0.0;
    Kd_jt << 1e0, 1e0, 1e0;
    AddJointTrackTask(1e0, Kp_jt, Kd_jt);
    SetupOSC();

    SetTorqueWeight(1e-6);

    // Ramp up torque initially
    StartTorqueRampUp(1e0);
}

int ArmOSC::UpdateControl() {
    // Update any tasks or objectives
    Eigen::Vector3d r(0.0, 1.0, 1.0);
    GetEndEffectorTaskMap()["tip"]->SetReference(r);

    // Compute IK
    // Eigen::VectorXd q(3);
    // q << 0.5, 0.5, 0.5;
    // UpdateJointTrackReference(q);

    // Run the operational space controller
    RunOSC();
    return 0;
}

int ArmOSC::MapMujocoState(const double *q, const double *v) {
    qpos() << q[0], q[1], q[2];
    qvel() << v[0], v[1], v[2];
    return 0;
}

void ArmOSC::UpdateDynamics() {
    LOG(INFO) << "UpdateDynamics";
    // Compute dynamic matrices and nullspace projectors
    const double *in[] = {qpos_.data(), qvel_.data()};
    double *out[3];
    out[0] = h_.data();
    arm_bias_vector(in, out, NULL, NULL, 0);
    out[0] = M_.data();
    arm_mass_matrix(in, out, NULL, NULL, 0);
    out[0] = B_.data();
    arm_actuation_matrix(in, out, NULL, NULL, 0);

    DynamicsQaccJacobian() = M_;
    DynamicsCtrlJacobian() = -B_;
    for (const auto &ee : GetEndEffectorTaskMap()) {
        DynamicsLambdaJacobian().middleCols(3 * ee.second->GetId(), 3) = -ee.second->J().transpose();
    }
    DynamicsConstraintVector() = -h_;
}
