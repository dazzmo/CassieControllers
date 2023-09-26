#include "os_ctrl_leg.h"

int CassieLegOSController::InitMatrices() {
    LOG(INFO) << "InitMatrices";
    M_ = Eigen::MatrixXd::Zero(nv_, nv_);
    Minv_ = Eigen::MatrixXd::Zero(nv_, nv_);
    J_ = Eigen::MatrixXd::Zero(1, nv_);
    dJdq_ = Eigen::MatrixXd::Zero(1, 1);
    JMJT_ = Eigen::MatrixXd::Zero(1, 1);
    B_ = Eigen::MatrixXd::Zero(nv_, nu_);
    N_ = Eigen::MatrixXd::Zero(nv_, nv_);

    h_ = Eigen::VectorXd::Zero(nv_);
    h_spring_ = Eigen::VectorXd::Zero(nv_);
    return 0;
}

int CassieLegOSController::SetupEndEffectors() {
    LOG(INFO) << "SetupEndEffectors";
    RegisterEndEffector("foot_front", cassie_foot_front);
    RegisterEndEffector("foot_back", cassie_foot_back);
    return 0;
}

void CassieLegOSController::UpdateDynamics() {
    LOG(INFO) << "UpdateDynamics";
    // Compute dynamic matrices and nullspace projectors
    const double *in[] = {qpos_.data(), qvel_.data()};
    double *out[3];
    out[0] = h_.data();
    cassie_bias_vector(in, out, NULL, NULL, 0);
    out[0] = h_spring_.data();
    cassie_spring_forces(in, out, NULL, NULL, 0);
    out[0] = M_.data();
    cassie_mass_matrix(in, out, NULL, NULL, 0);
    out[0] = Minv_.data();
    cassie_mass_matrix_inv(in, out, NULL, NULL, 0);
    out[0] = JMJT_.data();
    cassie_heel_spring_constraint_inertia(in, out, NULL, NULL, 0);
    out[0] = &c_;
    out[1] = J_.data();
    out[2] = dJdq_.data();
    cassie_heel_spring_constraint(in, out, NULL, NULL, 0);
    out[0] = B_.data();
    cassie_actuation(in, out, NULL, NULL, 0);

    // Compute null space matrix
    N_ = (Eigen::MatrixXd::Identity(nv_, nv_) -
          J_.transpose() * JMJT_.completeOrthogonalDecomposition().pseudoInverse() * J_ * Minv_)
             .transpose();

    // Dynamic constraint jacobians and vector
    DynamicsQaccJacobian() = M_;
    DynamicsCtrlJacobian() = -N_ * B_;
    for (const auto &ee : GetEndEffectorTaskMap()) {
        DynamicsLambdaJacobian().middleCols(3 * ee.second->GetId(), 3) = -N_ * ee.second->J().transpose();
    }
    DynamicsConstraintVector() = N_ * (h_ - h_spring_) - J_.transpose() * JMJT_.completeOrthogonalDecomposition().pseudoInverse() * dJdq_;
}

int CassieLegOSController::HeelSpringDeflection() {
    Eigen::MatrixXd J(1, 1);
    Eigen::VectorXd e(1), dJdq(1);
    Eigen::VectorXd hs(1);
    Eigen::VectorXd qj = qpos();

    const double *in[] = {qj.data()};
    double *out[3];
    out[0] = e.data();
    out[1] = J.data();
    out[2] = dJdq.data();

    std::cout << "qj: " << qj.transpose() << std::endl;

    const int max_iter = 50;
    for (int i = 0; i < max_iter; i++) {
        // Perform least-squares estimation
        cassie_heel_spring_constraint(in, out, NULL, NULL, 0);
        // Update
        hs -= (J.transpose() * J).inverse() * J.transpose() * e;
        // Update heel spring estimates in joint vector
        qj[5] = hs[0];

        if (e.lpNorm<Eigen::Infinity>() < 1e-4) {
            return 0;
        }
    }

    return 1;
}

int CassieLegOSController::MapMujocoState(const double *q, const double *v) {
    qpos() << q[7], q[8], q[9], q[14], q[15], q[16], q[17], q[20];
    qvel() << v[6], v[7], v[8], v[12], v[13], v[14], v[15], v[19];
    return 0;
}
