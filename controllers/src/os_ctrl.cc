#include "controllers/os_ctrl.h"

OperationalSpaceController::OperationalSpaceController() : Controller() {
    nc_ = 0;
}

int OperationalSpaceController::RegisterTask(const char* name, f_casadi_cg callback) {
    tasks_[name] = std::shared_ptr<Task>(new Task(3, nv_, callback));
    return 0;
}

int OperationalSpaceController::RegisterEndEffector(const char* name, f_casadi_cg callback) {
    ee_tasks_[name] = std::shared_ptr<EndEffectorTask>(new EndEffectorTask(nv_, callback));
    ee_tasks_[name]->SetId(nc_);
    nc_++;
    return 0;
}

int OperationalSpaceController::UpdateEndEffectorTasks() {
    for (auto const& task : ee_tasks_) {
        task.second->UpdateTask(qpos_, qvel_, true);
    }

    return 0;
}

int OperationalSpaceController::SetContact(const char* name, double mu, const Eigen::Vector3d& normal) {
    ee_tasks_[name]->inContact = true;
    ee_tasks_[name]->SetFrictionCoefficient(mu);
    ee_tasks_[name]->normal() = normal;
    return 0;
}

int OperationalSpaceController::RemoveContact(const char* name) {
    ee_tasks_[name]->inContact = false;
    return 0;
}

int OperationalSpaceController::AddQposReference(const Eigen::VectorXd& qpos) {
    return 0;
}

/**
 * @brief Sets up the Operational Space Controller (OSC) program by initialising the
 * necessary matrices and vectors for computation.
 *
 * @return int
 */
int OperationalSpaceController::SetupOSC() {
    LOG(INFO) << "OperationalSpaceController::InitProgram";
    LOG(INFO) << "starting";
    // Number of optimisation variables
    int nx = nv_ + 3 * nc_ + nu_;

    LOG(INFO) << "dynamic matrices";
    // Initialise dynamic components
    dyn_b_ = Eigen::VectorXd::Zero(nv_);

    LOG(INFO) << "qp matrices and vectors";
    H_ = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nx, nx);
    g_ = Eigen::VectorXd::Zero(nx);
    LOG(INFO) << "A";
    A_ = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nv_ + 4 * nc_, nx);
    lbA_ = -qpOASES::INFTY * Eigen::VectorXd::Ones(nv_ + 4 * nc_);
    ubA_ = qpOASES::INFTY * Eigen::VectorXd::Ones(nv_ + 4 * nc_);
    LOG(INFO) << "x";
    lbx_ = -qpOASES::INFTY * Eigen::VectorXd::Ones(nx);
    ubx_ = qpOASES::INFTY * Eigen::VectorXd::Ones(nx);
    x_ = Eigen::VectorXd::Zero(nx);

    LOG(INFO) << "friction cone approximations";
    for (auto const& ee : ee_tasks_) {
        // Friction cone constraints
        // TODO: Will need to account for surface normals
        A_.middleRows(nv_ + 4 * ee.second->GetId(), 4)
                .middleCols(nv_ + 3 * ee.second->GetId(), 3)
            << sqrt(2), 0.0, -ee.second->mu(),
            -sqrt(2), 0.0, -ee.second->mu(),
            0.0, sqrt(2), -ee.second->mu(),
            0.0, -sqrt(2), -ee.second->mu();
        ubA_.middleRows(nv_ + 4 * ee.second->GetId(), 4).setConstant(0.0);
        lbA_.middleRows(nv_ + 4 * ee.second->GetId(), 4).setConstant(-qpOASES::INFTY);
    }

    LOG(INFO) << "control bounds";
    for (int i = 0; i < nu_; i++) {
        ubx_[nv_ + 3 * nc_ + i] = u_max_[i];
        lbx_[nv_ + 3 * nc_ + i] = -u_max_[i];
    }

    // Create program
    qp_ = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(nx, nv_ + 4 * nc_));

    LOG(INFO) << "finished";
    return 0;
}

/**
 * @brief Solves the current OSC program with the current values for the state of the
 * system at time t
 *
 * @param t Current time for the controller
 * @return int
 */
const Eigen::VectorXd& OperationalSpaceController::RunOSC() {
    LOG(INFO) << "OperationalSpaceController::ComputeControl";
    UpdateEndEffectorTasks();
    UpdateDynamics();

    // Dynamic constraints
    lbA_.topRows(nv_) = DynamicsConstraintVector();
    ubA_.topRows(nv_) = DynamicsConstraintVector();

    // Reset cost
    H_.setZero();
    g_.setZero();

    // Create objective
    LOG(INFO) << "end effector tasks";
    for (auto const& ee : ee_tasks_) {
        Eigen::DiagonalMatrix<double, -1> Kp(3), Kd(3);
        Kp.diagonal() << 5e2, 5e2, 5e2;
        Kd.diagonal() << 1e1, 1e1, 1e1;

        Eigen::Vector3d e = ee.second->x() - ee.second->r();
        Eigen::Vector3d de = ee.second->dx() - ee.second->dr();
        Eigen::Vector3d a = -ee.second->dJdv() - Kp * e - Kd * de;

        LOG(INFO) << "Error: e = " << e.transpose() << " de = " << de.transpose();
        LOG(INFO) << "Task Acceleration = " << (ee.second->J() * qacc() + ee.second->dJdv()).transpose() << " a = " << a.transpose();

        Eigen::DiagonalMatrix<double, 3> W;
        W.diagonal() << 1e3, 1e3, 1e3;
        // Add to objective
        H_.topLeftCorner(nv_, nv_) += ee.second->J().transpose() * W * ee.second->J();
        g_.topRows(nv_) -= 2.0 * ee.second->J().transpose() * W * a;

        // Contact
        if (ee.second->inContact) {
            ubx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << qpOASES::INFTY, qpOASES::INFTY, qpOASES::INFTY;
            lbx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << -qpOASES::INFTY, -qpOASES::INFTY, qpOASES::ZERO;
        } else {
            ubx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << qpOASES::ZERO, qpOASES::ZERO, qpOASES::ZERO;
            lbx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << -qpOASES::ZERO, -qpOASES::ZERO, -qpOASES::ZERO;
        }
    }

    // Encourage target pose
    // Eigen::DiagonalMatrix<double, -1> Kp(3), Kd(3);
    // Kp.diagonal() << 5e1, 5e1, 5e1;
    // Kd.diagonal() << 1e1, 1e1, 1e1;

    // Eigen::DiagonalMatrix<double, 3> W;
    // W.diagonal() << 1e3, 1e3, 1e3;

    // // Pose jacobian is the identity matrix

    // // Add to objective
    // H_.topLeftCorner(nv_, nv_) += ee.second->J().transpose() * W * ee.second->J();
    // g_.topRows(nv_) -= 2.0 * ee.second->J().transpose() * W * a;

    // Update acceleration bounds (for joint/velocity damping)
    Eigen::VectorXd qacc_u(nv_), qacc_l(nv_);
    GetAccelerationLimits(qpos(), qvel(), qpos_bl(), qpos_bu(), qvel_max(), qacc_max_,
                          qacc_l, qacc_u, nq_, nv_, 1.0 / freq_);

    LOG(INFO) << "ql: " << qpos_bl().transpose();
    LOG(INFO) << "q: " << qpos().transpose();
    LOG(INFO) << "qu: " << qpos_bu().transpose();
    LOG(INFO) << "v: " << qvel().transpose();
    LOG(INFO) << "vmax: " << qvel_max().transpose();
    LOG(INFO) << "al: " << qacc_l.transpose();
    LOG(INFO) << "au: " << qacc_u.transpose();

    for (int i = 0; i < nv_; ++i) {
        lbx_[i] = qacc_l[i];
        ubx_[i] = qacc_u[i];
    }

    // Control weighting
    Eigen::DiagonalMatrix<double, -1> Wu(nu_);
    Wu.diagonal() << 1e-2 * u_max_.cwiseInverse().cwiseAbs2();

    for (int i = 0; i < nu_; ++i) {
        H_(nv_ + 3 * nc_ + i, nv_ + 3 * nc_ + i) = Wu.diagonal()[i];
        // Keep torque similar to previous output
        if (!first_solve_) {
            g_.middleRows(nv_ + 3 * nc_, nu_) = -2.0 * Wu * ctrl();
        }
    }

    // Acceleration weighting
    // Eigen::DiagonalMatrix<double, -1> Wa(nv_);
    // Wa.diagonal() << 1e0, 1e0, 1e0, 1e0, 1e2, 1e0, 1e2, 1e4;
    // for (int i = 0; i < nv_; ++i) {
    //     H_(i, i) += Wa.diagonal()[i];
    // }

    LOG(INFO) << "solve";
    int nWSR = 1000;
    // LOG(INFO) << "H: " << H_;
    // LOG(INFO) << "g: " << g_;
    // LOG(INFO) << "A: " << A_;
    // LOG(INFO) << "lbA: " << lbA_;
    // LOG(INFO) << "ubA: " << ubA_;
    // LOG(INFO) << "lbx: " << lbx_;
    // LOG(INFO) << "ubx: " << ubx_;

    H_ *= 2.0;
    qpOASES::returnValue status = qpOASES::SUCCESSFUL_RETURN;

    if (first_solve_) {
        status = qp_->init(H_.data(), g_.data(), A_.data(), lbx_.data(), ubx_.data(), lbA_.data(), ubA_.data(), nWSR);
        first_solve_ = false;
    } else {
        status = qp_->hotstart(H_.data(), g_.data(), A_.data(), lbx_.data(), ubx_.data(), lbA_.data(), ubA_.data(), nWSR);
    }

    if (status != qpOASES::SUCCESSFUL_RETURN) {
        LOG(ERROR) << "Failed";
        while (1);
    }
    // Get solution
    qp_->getPrimalSolution(x_.data());
    qacc_ = x_.topRows(nv_).transpose();
    LOG(INFO) << "qacc: " << qacc_.transpose();
    // Extract solution components
    u_ = ApplyTorquePreScale() * x_.bottomRows(nu_);
    LOG(INFO) << "u: " << u_.transpose();
    for (auto const& ee : ee_tasks_) {
        ee.second->lambda() = x_.middleRows(nv_ + 3 * ee.second->GetId(), 3);
        LOG(INFO) << "lambda (" << ee.first << "): " << ee.second->lambda().transpose();
    }

    return u_;
}
