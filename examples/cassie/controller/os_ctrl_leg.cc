#include "os_ctrl_leg.h"

void CassieLegOSC::SetupController() {
    LOG(INFO) << "InitMatrices";

    M_ = Eigen::MatrixXd::Zero(nv_, nv_);
    Minv_ = Eigen::MatrixXd::Zero(nv_, nv_);
    J_ = Eigen::MatrixXd::Zero(1, nv_);
    dJdq_ = Eigen::VectorXd::Zero(1, 1);
    JMJT_ = Eigen::MatrixXd::Zero(1, 1);
    B_ = Eigen::MatrixXd::Zero(nv_, nu_);
    N_ = Eigen::MatrixXd::Zero(nv_, nv_);

    h_ = Eigen::VectorXd::Zero(nv_);
    h_spring_ = Eigen::VectorXd::Zero(nv_);

    qpos_bl() << -0.3927, -0.3927, -0.8727, -2.8623, -0.3, 0.75, -0.3, -2.4435;
    qpos_bu() << 0.3927, 0.3927, 1.3963, -0.95, 0.3, 3.0, 0.3, -0.5236;
    ctrl_max() << 4.5, 4.5, 12.2, 12.2, 0.9;

    SetupEndEffectors();
    SetupOSC();

    // Ramp up torque initially
    StartTorqueRampUp(1e-1);
}

int CassieLegOSC::SetupEndEffectors() {
    LOG(INFO) << "SetupEndEffectors";
    RegisterEndEffector("ankle", cassie_ankle);
    // RegisterEndEffector("foot_front", cassie_foot_front);
    // RegisterEndEffector("foot_back", cassie_foot_back);
    return 0;
}

int CassieLegOSC::UpdateControl() {
    // Update any tasks or objectives

    // Run the operational space controller
    RunOSC();
    return 0;
}

void CassieLegOSC::UpdateDynamics() {
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
    out[0] = &c_;
    out[1] = J_.data();
    out[2] = dJdq_.data();
    cassie_heel_spring_constraint(in, out, NULL, NULL, 0);
    out[0] = B_.data();
    cassie_actuation(in, out, NULL, NULL, 0);

    JMJT_ = J_ * M_.inverse() * J_.transpose();

    // LOG(INFO) << "M: " << M_;
    // LOG(INFO) << "h: " << h_;
    // LOG(INFO) << "h_spring: " << h_spring_;
    // LOG(INFO) << "J: " << J_;
    // LOG(INFO) << "JMJT: " << JMJT_;
    // LOG(INFO) << "pinv(JMJT): " << JMJT_.completeOrthogonalDecomposition().pseudoInverse();
    // LOG(INFO) << "B: " << B_;

    // Compute null space matrix
    N_ = Eigen::MatrixXd::Identity(nv_, nv_) - J_.transpose() * JMJT_.completeOrthogonalDecomposition().pseudoInverse() * J_ * M_.inverse();

    // Dynamic constraint jacobians and vector
    DynamicsQaccJacobian() = M_;
    DynamicsCtrlJacobian() = -N_ * B_;
    for (const auto &ee : GetEndEffectorTaskMap()) {
        DynamicsLambdaJacobian().middleCols(3 * ee.second->GetId(), 3) = -N_ * ee.second->J().transpose();
    }
    DynamicsConstraintVector() = N_ * (h_spring_ - h_) - J_.transpose() * JMJT_.completeOrthogonalDecomposition().pseudoInverse() * dJdq_;
}

int CassieLegOSC::HeelSpringDeflection() {
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

/**
 * @brief Performs inverse-kinematics with nonlinear equality constraints 
 * 
 * @param qpos 
 * @param x 
 * @param q0 
 * @return int 
 */
int CassieLegOSC::InverseKinematics(const Eigen::VectorXd &qpos, const Eigen::Vector3d &x_d, const Eigen::VectorXd &q0) {
    LOG(INFO) << "Starting IK";
    // Solver options
    const int max_it = 50;
    // Iterates
    Eigen::VectorXd q_i = q0;
    Eigen::VectorXd lamba_i(1);
    Eigen::Vector3d x_i;
    // Gradients, Jacobians and Hessians
    Eigen::MatrixXd J(3, 8);
    // Lagrangian Hessian
    Eigen::MatrixXd lag_hess(nq_, nq_);

    J.setZero();
    // Constrained Gauss-Newton approach for solving quality-constrained least-squares
    Eigen::VectorXd v, x, lam(1);
    Eigen::MatrixXd jac_kkt(nq_ + 1, nq_ + 1);
    Eigen::VectorXd vec_kkt(nq_ + 1);
    jac_kkt.setZero(); 
    vec_kkt.setZero(); 

    const double eps = 1e-3;

    Eigen::VectorXd gi(1);
    Eigen::MatrixXd jac_gi(1, nq_), hess_gi(nq_, nq_);
    LOG(INFO) << "qi: " << q_i.transpose();
    const double *in[] = {q_i.data(), nullptr};
    double *out_ankle[3];
    out_ankle[0] = x_i.data();
    out_ankle[1] = J.data();
    out_ankle[2] = NULL;

    double *out_constraint[4];
    out_constraint[0] = gi.data();
    out_constraint[1] = jac_gi.data();
    out_constraint[2] = NULL;
    out_constraint[3] = hess_gi.data();

    // Damping for least squares
    Eigen::Vector3d damp(1e2, 1e2, 1e2);
    
    // Identity matrix 
    Eigen::MatrixXd I(nq_, nq_);
    I.setIdentity();

    lam.setOnes();

    // TODO: Don't rely on spring deflection as much (add penalisation)

    for (int i = 0; i < max_it; i++) {
        LOG(INFO) << "Iteration " << i;
        // Compute end-effector jacobian
        cassie_ankle(in, out_ankle, NULL, NULL, 0);
        LOG(INFO) << "Ankle data computed";
        // Compute constraint jacobian and hessian
        cassie_heel_spring_constraint(in, out_constraint, NULL, NULL, 0);
        LOG(INFO) << "Constraint data computed";

        // Compute FK error
        LOG(INFO) << "xi = " << x_i.transpose();
        Eigen::VectorXd e = x_i - x_d;
        LOG(INFO) << "e = " << e.transpose();

        if(e.squaredNorm() < eps) break;
        // Compute hessian of lagrangian
        lag_hess = (J.transpose() * J + 1e-2 * I) - lam[0] * hess_gi;
        LOG(INFO) << "lag_hess = " << lag_hess;

        // Compute KKT jacobian
        LOG(INFO) << "jac_kkt = " << jac_kkt;

        jac_kkt.topLeftCorner(nq_, nq_) << lag_hess;
        jac_kkt.bottomLeftCorner(1, nq_) << jac_gi;
        jac_kkt.topRightCorner(nq_, 1) << jac_gi.transpose();

        LOG(INFO) << "jac_kkt = " << jac_kkt;

        // Compute KKT vector
        vec_kkt << J.transpose() * e, gi;

        // Compute step
        Eigen::VectorXd dx = jac_kkt.ldlt().solve(-vec_kkt);

        LOG(INFO) << "dx: " << dx.transpose();

        // Take a step
        q_i += dx.topRows(nq_); 
        lam += dx.bottomRows(1); 

        // Clamp results
        for(int i = 0 ; i < nq_; i++) {
            q_i[i] = std::min(std::max(q_i[i], qpos_bl_[i]), qpos_bu_[i]);
        }

    }

    LOG(INFO) << "q: " << q_i.transpose();

    return 0;
}

int CassieLegOSC::MapMujocoState(const double *q, const double *v) {
    qpos() << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13];
    qvel() << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12];
    return 0;
}
