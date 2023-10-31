#include "osc_leg.h"

void CassieLegOSC::SetupController() {
    LOG(INFO) << "InitMatrices";

    M_ = Eigen::MatrixXd::Zero(CASSIE_LEG_NV, CASSIE_LEG_NV);
    J_ = Eigen::MatrixXd::Zero(1, CASSIE_LEG_NV);
    dJdq_ = Eigen::VectorXd::Zero(1, 1);
    JMJT_ = Eigen::MatrixXd::Zero(1, 1);
    B_ = Eigen::MatrixXd::Zero(CASSIE_LEG_NV, CASSIE_LEG_NU);
    N_ = Eigen::MatrixXd::Zero(CASSIE_LEG_NV, CASSIE_LEG_NV);

    h_ = Eigen::VectorXd::Zero(CASSIE_LEG_NV);
    h_spring_ = Eigen::VectorXd::Zero(CASSIE_LEG_NV);

    qpos_0() << 0.00449956, 0, 0.497301, -1.1997, 0, 1.42671, 0.0, -1.59681;
    qpos_bl() << -0.3927, -0.3927, -0.8727, -2.8623, -0.3, 0.75, -0.3, -2.4435;
    qpos_bu() << 0.3927, 0.3927, 1.3963, -0.95, 0.3, 3.0, 0.3, -0.5236;
    ctrl_max() << 4.5, 4.5, 12.2, 12.2, 0.9;
    qvel_max().setConstant(1e1);
    qacc_max().setConstant(1e20);

    SetupEndEffectors();
    CreateOSC();

    // Set weights for controller
    Eigen::VectorXd Kp(CASSIE_LEG_NQ), Kd(CASSIE_LEG_NV), w(CASSIE_LEG_NQ);
    Kp.setConstant(2e2);
    Kd.setConstant(5e0);
    w.setConstant(1e3);

    w.setZero();
    w[7] = 1.0;

    UpdateJointTrackPDGains(Kp, Kd);
    UpdateJointTrackWeighting(w);

    // Ramp up torque initially
    StartTorqueRampUp(1e0);
}

int CassieLegOSC::SetupEndEffectors() {
    LOG(INFO) << "SetupEndEffectors";
    RegisterEndEffector("ankle", cassie_ankle);
    GetEndEffectorTaskMap()["ankle"]->SetTaskWeighting(Eigen::Vector3d(1e-6, 1e-6, 1e-6));
    GetEndEffectorTaskMap()["ankle"]->SetProportionalErrorGain(Eigen::Vector3d(1e1, 1e1, 1e1));
    GetEndEffectorTaskMap()["ankle"]->SetDerivativeErrorGain(Eigen::Vector3d(1e0, 1e0, 1e0));

    // RegisterEndEffector("foot_front", cassie_foot_front);
    // RegisterEndEffector("foot_back", cassie_foot_back);
    return 0;
}

int CassieLegOSC::UpdateControl() {
    // Update any tasks or objectives
    Eigen::Vector3d r(0.0, 0.0, -0.8);
    GetEndEffectorTaskMap()["ankle"]->SetReference(r);

    // Compute IK for foot
    Eigen::VectorXd q(CASSIE_LEG_NQ);
    InverseKinematics(q, r, qpos_0());
    
    q.setZero();
    std::cout << "Time: " << t_;
    q[7] = 0.3 + 0.3 * sin(5 * t_);
    UpdateJointTrackReference(q);

    // Run the operational space controller
    RunOSC();
    return 0;
}

void CassieLegOSC::UpdateDynamics() {
    LOG(INFO) << "UpdateDynamics";
    // Compute dynamic matrices and nullspace projectors
    const double *in[] = {qpos_.data(), qvel_.data()};
    double *out[4] = {NULL, NULL, NULL, NULL};
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

    // Compute null space matrix
    N_ = Eigen::MatrixXd::Identity(CASSIE_LEG_NV, CASSIE_LEG_NV) - J_.transpose() * JMJT_.completeOrthogonalDecomposition().pseudoInverse() * J_ * M_.inverse();

    // Dynamic constraint jacobians and vector
    DynamicsQaccJacobian() = M_;
    DynamicsCtrlJacobian() = -N_ * B_;
    for (const auto &ee : GetEndEffectorTaskMap()) {
        DynamicsLambdaJacobian().middleCols(3 * ee.second->GetId(), 3) = -N_ * ee.second->J().transpose();
    }
    DynamicsConstraintVector() = N_ * (h_spring_ - h_) - J_.transpose() * JMJT_.completeOrthogonalDecomposition().pseudoInverse() * dJdq_;

    LOG(INFO) << "UpdateDynamics finished";
}

/**
 * @brief Estimates the angle of the heel-spring based on the configuration of the leg, given no encoder is provided for the
 * heel spring joint.
 *
 * @return int
 */
int CassieLegOSC::HeelSpringDeflection() {
    Eigen::MatrixXd J(1, 1);
    Eigen::VectorXd e(1), dJdq(1);
    Eigen::VectorXd hs(1);
    Eigen::VectorXd qj = qpos();

    const double *in[] = {qj.data()};
    double *out[3] = {e.data(), J.data(), dJdq.data()};

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
int CassieLegOSC::InverseKinematics(Eigen::VectorXd &qpos, const Eigen::Vector3d &x_d, const Eigen::VectorXd &q0) {
    LOG(INFO) << "Starting IK";
    // Solver options
    const int max_it = 50;
    // Iterates
    Eigen::VectorXd q_i(nq_);
    if (ik_restart_) {
        q_i = qpos_0_;
    } else {
        q_i = q0;
    }

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

    double *out_ankle[3] = {x_i.data(), J.data(), NULL};
    double *out_constraint[4] = {gi.data(), jac_gi.data(), NULL, hess_gi.data()};

    // Identity matrix
    Eigen::MatrixXd I(nq_, nq_);
    I.setZero();
    I.diagonal() << 1e-2, 1e-2, 1e-2, 1e-2, 1e1, 1e-2, 1e1, 1e-2;

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

        if (e.squaredNorm() < eps) break;
        // Compute hessian of lagrangian
        lag_hess = (J.transpose() * J + I) - lam[0] * hess_gi;
        // LOG(INFO) << "lag_hess = " << lag_hess;

        // Compute KKT jacobian
        // LOG(INFO) << "jac_kkt = " << jac_kkt;

        jac_kkt.topLeftCorner(nq_, nq_) << lag_hess;
        jac_kkt.bottomLeftCorner(1, nq_) << jac_gi;
        jac_kkt.topRightCorner(nq_, 1) << jac_gi.transpose();

        // LOG(INFO) << "jac_kkt = " << jac_kkt;

        // Compute KKT vector
        vec_kkt << J.transpose() * e, gi;

        // Compute step
        Eigen::VectorXd dx = jac_kkt.ldlt().solve(-vec_kkt);

        LOG(INFO) << "dx: " << dx.transpose();

        // Take a step
        q_i += dx.topRows(nq_);
        lam += dx.bottomRows(1);

        // Clamp results
        for (int i = 0; i < nq_; i++) {
            q_i[i] = std::min(std::max(q_i[i], qpos_bl_[i]), qpos_bu_[i]);
        }
    }

    LOG(INFO) << "q: " << q_i.transpose();
    qpos = q_i;

    return 0;
}

int CassieLegOSC::MapMujocoState(const double *q, const double *v) {
    qpos() << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13];
    qvel() << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12];
    return 0;
}
