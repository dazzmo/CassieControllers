#include "controllers/os_ctrl.h"

OperationalSpaceController::OperationalSpaceController(int nq, int nv, int nu) : Controller(nq, nv, nu) {
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

int OperationalSpaceController::InitProgram() {
    LOG(INFO) << "OperationalSpaceController::InitProgram";
    LOG(INFO) << "starting";
    // Number of optimisation variables
    int nx = nv_ + 3 * nc_ + nu_;

    LOG(INFO) << "dynamic matrices";
    // Initialise dynamic components
    dyn_b_ = Eigen::VectorXd::Zero(nv_);

    LOG(INFO) << "qp matrices and vectors";
    H_ = Eigen::MatrixXd::Identity(nx, nx);
    g_ = Eigen::VectorXd::Zero(nx);

    A_ = Eigen::MatrixXd::Zero(nv_ + 4 * nc_, nx);
    lbA_ = Eigen::VectorXd::Zero(nv_ + 4 * nc_);
    ubA_ = Eigen::VectorXd::Zero(nv_ + 4 * nc_);

    lbx_ = Eigen::VectorXd::Zero(nx);
    ubx_ = Eigen::VectorXd::Zero(nx);

    LOG(INFO) << "friction cone approximations";
    for (auto const& ee : ee_tasks_) {
        // Friction cone constraints
        // TODO: Will need to account for surface normals
        A_.middleRows(nv_ + 4 * ee.second->GetId(), 4)
                .middleCols(nv_ + 3 * ee.second->GetId(), 3)
            << sqrt(2), 0.0, -ee.second->mu(),
            -sqrt(2), 0.0, ee.second->mu(),
            0.0, sqrt(2), -ee.second->mu(),
            0.0, -sqrt(2), ee.second->mu();
        lbA_.middleRows(nv_ + 4 * ee.second->GetId(), 4).setZero();
        ubA_.middleRows(nv_ + 4 * ee.second->GetId(), 4).setConstant(1e19);
    }

    LOG(INFO) << "control bounds";
    for (int i = 0; i < nu_; i++) {
        lbx_[nv_ + 3 * nc_ + i] = -u_max_[i];
        ubx_[nv_ + 3 * nc_ + i] = u_max_[i];
    }

    LOG(INFO) << "finished";
    return 0;
}

int OperationalSpaceController::ComputeControl() {
    LOG(INFO) << "OperationalSpaceController::ComputeControl";
    UpdateEndEffectorTasks();
    UpdateDynamics();

    // Dynamic constraints
    lbA_.topRows(nv_) = DynamicsConstraintVector();
    ubA_.topRows(nv_) = DynamicsConstraintVector();

    LOG(INFO) << "tasks";
    for (auto const& task : tasks_) {
        // Add to objective
        H_.topLeftCorner(nv_, nv_) += task.second->task_weight * task.second->J().transpose() * task.second->J();
        g_.topRows(nv_) += task.second->task_weight * (task.second->dJdv() - task.second->ddx()).transpose() * task.second->J();
    }

    // Create objective
    LOG(INFO) << "end effector tasks";
    for (auto const& ee : ee_tasks_) {
        // Compute ee Jacobian
        int n;
        LOG(INFO) << "name: " << ee.first;
        // Add to objective
        LOG(INFO) << "H";
        H_.topLeftCorner(nv_, nv_) += ee.second->J().transpose() * ee.second->J();
        LOG(INFO) << "g";
        g_.topRows(nv_) += ee.second->J().transpose() * (ee.second->dJdv() - ee.second->ddx());

        // Contact
        LOG(INFO) << "inContact";
        if (ee.second->inContact) {
            ubx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << 1e19, 1e19, 1e19;
            lbx_.middleRows(nv_ + 3 * ee.second->GetId(), 3) << -1e19, -1e19, 0.0;
        } else {
            lbx_.middleRows(nv_ + 3 * ee.second->GetId(), 3).setZero();
            ubx_.middleRows(nv_ + 3 * ee.second->GetId(), 3).setZero();
        }
    }

    LOG(INFO) << "solve";
    int nWSR = 500;
    if (1) {
        qp_.init(H_.data(), g_.data(), A_.data(), lbx_.data(), ubx_.data(), lbA_.data(), ubA_.data(), nWSR);
    } else {
        qp_.hotstart(H_.data(), g_.data(), A_.data(), lbx_.data(), ubx_.data(), lbA_.data(), ubA_.data(), nWSR);
    }

    LOG(INFO) << "getting solution";
    // Get solution
    qp_.getPrimalSolution(x_.data());

    // Extract solution components
    u_ = x_.bottomRows(nu_);
    for (auto const& ee : ee_tasks_) {
        ee.second->lambda() = x_.middleRows(nv_ + 3 * ee.second->GetId(), 3);
    }

    return 0;
}
