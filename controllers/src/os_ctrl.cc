#include "controllers/os_ctrl.h"

OperationalSpaceController::OperationalSpaceController(int nq, int nv, int nu) : Controller(nq, nv, nu) {
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

int OperationalSpaceController::UpdateJointTrackReference(const Eigen::VectorXd& qpos_r) {
    joint_track_task_->SetReference(qpos_r);
    return 0;
}

int OperationalSpaceController::UpdateJointTrackReference(const Eigen::VectorXd& qpos_r, const Eigen::VectorXd& qvel_r) {
    joint_track_task_->SetReference(qpos_r, qvel_r);
    return 0;
}

int OperationalSpaceController::UpdateJointTrackWeighting(const Eigen::VectorXd& w) {
    joint_track_task_->SetTaskWeighting(w);
    return 0;
}

int OperationalSpaceController::UpdateJointTrackPDGains(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd) {
    joint_track_task_->SetProportionalErrorGain(Kp);
    joint_track_task_->SetDerivativeErrorGain(Kd);
    return 0;
}

int OperationalSpaceController::UpdateJointLimitWeighting(const Eigen::VectorXd& w) {
    joint_limits_task_->SetTaskWeighting(w);
    return 0;
}

int OperationalSpaceController::UpdateJointLimitPDGains(const Eigen::VectorXd& Kp, const Eigen::VectorXd& Kd) {
    joint_limits_task_->SetProportionalErrorGain(Kp);
    joint_limits_task_->SetDerivativeErrorGain(Kd);
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

    // Create joint track task
    joint_track_task_ = new JointTrackTask(nq_, nv_);

    // Create joint limit avoidance task
    joint_limits_task_ = new JointLimitsTask(nq_, nv_);
    joint_limits_task_->SetUpperPositionLimit(qpos_bu());
    joint_limits_task_->SetLowerPositionLimit(qpos_bl());

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
            << sqrt(2),
            0.0, -ee.second->mu(),
            -sqrt(2), 0.0, -ee.second->mu(),
            0.0, sqrt(2), -ee.second->mu(),
            0.0, -sqrt(2), -ee.second->mu();
        ubA_.middleRows(nv_ + 4 * ee.second->GetId(), 4).setConstant(0.0);
        lbA_.middleRows(nv_ + 4 * ee.second->GetId(), 4).setConstant(-qpOASES::INFTY);
    }

    LOG(INFO) << "acceleration bounds";
    for (int i = 0; i < nv_; i++) {
        ubx_[i] = qacc_max_[i];
        lbx_[i] = -qacc_max_[i];
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
    double c = 0.0;
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
        ee.second->PrintTaskData();
        // Task jacobian in qacc
        const Eigen::MatrixXd& A = ee.second->J();
        // Task weight
        const Eigen::Matrix3d W = ee.second->weight().asDiagonal();
        // Task constant vector
        Eigen::Vector3d a = ee.second->dJdq() + ee.second->TaskErrorPD();

        // Add to objective
        // H_.topLeftCorner(nv_, nv_) += A.transpose() * W * A;
        // g_.topRows(nv_) += 2.0 * A.transpose() * W * a;
        // c += (W * a).dot(a);

        // Contact
        if (ee.second->inContact) {
            ubx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << qpOASES::INFTY, qpOASES::INFTY, qpOASES::INFTY;
            lbx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << -qpOASES::INFTY, -qpOASES::INFTY, qpOASES::ZERO;
        } else {
            ubx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << qpOASES::ZERO, qpOASES::ZERO, qpOASES::ZERO;
            lbx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << -qpOASES::ZERO, -qpOASES::ZERO, -qpOASES::ZERO;
        }
    }

    // Joint tracking
    LOG(INFO) << "joint tracking";
    joint_track_task_->UpdateTask(qpos(), qvel());
    joint_track_task_->PrintTaskData();
    const Eigen::MatrixXd& At = joint_track_task_->J();
    const Eigen::MatrixXd Wt = joint_track_task_->weight().asDiagonal();
    Eigen::VectorXd at = joint_track_task_->TaskErrorPD();
    // Add to objective
    H_.topLeftCorner(nv_, nv_) += At.transpose() * Wt * At;
    g_.topRows(nv_) += 2.0 * At.transpose() * Wt * at;
    c += (Wt * at).dot(at);

    // Joint limit avoidance
    // LOG(INFO) << "joint limit avoidance";
    // LOG(INFO) << "qu: " << qpos_bu().transpose();
    // LOG(INFO) << " q: " << qpos().transpose();
    // LOG(INFO) << "ql: " << qpos_bl().transpose();
    // joint_limits_task_->UpdateTask(qpos(), qvel());
    // joint_limits_task_->PrintTaskData();
    // const Eigen::MatrixXd& Al = joint_limits_task_->J();
    // const Eigen::MatrixXd Wl = joint_limits_task_->weight().asDiagonal();
    // Eigen::VectorXd al = joint_limits_task_->TaskErrorPD();
    // // Add to objective
    // H_.topLeftCorner(nv_, nv_) += Al.transpose() * Al;
    // g_.topRows(nv_) += 2.0 * Al.transpose() * al;
    // c += (al).dot(al);

    // Torque regularisation
    for(int i = 0; i < nu_; ++i) {
        int idx = nv_ + 3 * nc_ + i;
        H_(idx, idx) += 1e-6;
    }

    LOG(INFO) << "solve";
    int nWSR = 1000;

    H_ *= 2.0;
    qpOASES::returnValue status = qpOASES::SUCCESSFUL_RETURN;

    if (first_solve_) {
        qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
        status = qp_->init(H_.data(), g_.data(), A_.data(), lbx_.data(), ubx_.data(), lbA_.data(), ubA_.data(), nWSR);
        first_solve_ = false;
    } else {
        status = qp_->hotstart(H_.data(), g_.data(), A_.data(), lbx_.data(), ubx_.data(), lbA_.data(), ubA_.data(), nWSR);
    }

    if (status != qpOASES::SUCCESSFUL_RETURN) {
        LOG(ERROR) << "Failed";
        while (1)
            ;
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

    // Joint track error
    LOG(INFO) << "Joint track error";
    LOG(INFO) << "e = " << -joint_track_task_->TaskErrorPD().transpose();
    LOG(INFO) << "ddx = " << (At * qacc_ + joint_track_task_->dJdq()).transpose();


    return u_;
}
