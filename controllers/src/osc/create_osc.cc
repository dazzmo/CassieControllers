#include "controllers/osc/osc.h"

using namespace controller::osc;

OperationalSpaceController::OperationalSpaceController() : Controller(), start_idx_() {
}

/**
 * @brief Adds a new task
 *
 * @param name
 * @param callback
 * @return int
 */
void OperationalSpaceController::AddTask(const std::string& name, Dimension n, Task::TaskCallbackFunction callback) {
    tasks_[name] = std::shared_ptr<Task>(new Task(name, n, m_->size(), callback));
}

/**
 * @brief Adds a three-dimensional end-effector task, where callback is a function with inputs
 * (q, v) and outputs the task, its jacobian and its time derivative-velocity product (i.e. (x, J, dJdq))
 *
 * @param name
 * @param callback
 * @return int
 */
void OperationalSpaceController::AddEndEffectorTask(const std::string& name, Task::TaskCallbackFunction& callback) {
    ee_tasks_[name] = std::shared_ptr<EndEffectorTask>(new EndEffectorTask(name, m_->size(), callback));
    // m_.nc++;
}

void OperationalSpaceController::SetEndEffectorContact(const std::string& name, double mu, const Eigen::Vector3d& normal) {
    ee_tasks_[name]->inContact = true;
    ee_tasks_[name]->SetFrictionCoefficient(mu);
    ee_tasks_[name]->normal() = normal;
}

void OperationalSpaceController::RemoveEndEffectorContact(const std::string& name) {
    ee_tasks_[name]->inContact = false;
}

void OperationalSpaceController::UpdateJointTrackReference(const Eigen::VectorXd& qpos_r) {
    if (use_joint_track_) {
        joint_track_task_->SetReference(qpos_r);
    }
}

void OperationalSpaceController::UpdateJointTrackReference(const Eigen::VectorXd& qpos_r, const Eigen::VectorXd& qvel_r) {
    if (use_joint_track_) {
        joint_track_task_->SetReference(qpos_r, qvel_r);
    }
}

/**
 * @brief Includes a task that encourages the state of the system to follow a provided trajectory in
 * q and/or v.
 *
 * @param w
 * @param Kp
 * @param Kd
 * @return int
 */
void OperationalSpaceController::AddJointTrackTask(double w,
                                                   const Eigen::VectorXd& Kp,
                                                   const Eigen::VectorXd& Kd) {
    if (joint_track_task_ == nullptr) {
        // Create joint limit avoidance task
        joint_track_task_ = new JointTrackTask(m_->size());

        joint_track_task_->SetTaskWeighting(w);
        joint_track_task_->SetErrorGains(Kp, Kd);

        use_joint_track_ = true;

        // Add to map of tasks
        tasks_[joint_track_task_->name()] = std::shared_ptr<Task>(joint_track_task_);
        LOG(INFO) << "Joint track task added";
    } else {
        throw std::runtime_error("AddJointTrackTask: tracking task already added!");
    }
}

void OperationalSpaceController::AddJointLimitsTask(double w,
                                                    const Eigen::VectorXd& Kp,
                                                    const Eigen::VectorXd& Kd) {
    if (joint_limits_task_ != nullptr) {
        // Create joint track task
        joint_track_task_ = new JointTrackTask(m_->size());
        joint_limits_task_->SetTaskWeighting(w);
        joint_limits_task_->SetErrorGains(Kp, Kd);

        joint_limits_task_->SetUpperPositionLimit(m_->bounds().qu);
        joint_limits_task_->SetLowerPositionLimit(m_->bounds().ql);

        use_joint_limits_ = true;

        tasks_[joint_limits_task_->name()] = std::shared_ptr<Task>(joint_limits_task_);

    } else {
        throw std::runtime_error("AddJointLimitsTask: limits task already added!");
    }
}

/**
 *
 * @brief Sets up the Operational Space Controller (OSC) program by initialising the
 * necessary matrices and vectors for computation.
 *
 * Quadratic programs created have the following variables for optimisation
 * If holonomic constraints are used explicitly (set by the options),
 * then we have x = [qacc, lambda_c, lambda_h, u]
 * If holonomic constraints are solved for implicitly,
 * then we have x = [qacc, lambda_c, u]
 *
 * @param opt
 * @return int
 */
void OperationalSpaceController::CreateOSC(const Options& opt) {
    LOG(INFO) << "starting";
    // Create options
    opt_ = new Options(opt);

    // Number of optimisation variables
    int nx = m_->size().nq + 3 * m_->nc_ + m_->size().nu;
    // Number of equality constraints
    int ng = m_->size().nq + 4 * m_->nc_;

    if (opt_->use_constraint_nullspace_projector) {
        nx += m_.ng;
        ng += m_.ng;
    }

    // Set up indices
    start_idx_.qacc = 0;
    start_idx_.lambda_c = m_.nv;
    if (opt_->use_constraint_nullspace_projector) {
        start_idx_.lambda_h = start_idx_.lambda_c + 3 * m_.nc;
        start_idx_.ctrl = start_idx_.lambda_h + m_.ng;
    } else {
        start_idx_.lambda_h = -1;
        start_idx_.ctrl = start_idx_.lambda_c + 3 * m_.nc;
    }

    // Create results
    res_ = new OptimisationResult(m_.nv, m_.nc, m_.ng, m_.nu);

    // Create program
    qp_data_ = new QPData(nx, ng);
    qp_ = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(nx, ng));
    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);

    // Create holonomic constraint jacobian
    Jh_ = Eigen::MatrixXd::Zero(m_.ng, m_.nv);
    dJhdq_ = Eigen::VectorXd::Zero(m_.ng);

    LOG(INFO) << "friction cone approximations";
    for (auto const& ee : ee_tasks_) {
        // Friction cone constraints
        // TODO: Will need to account for surface normals
        qp_data_->A.middleRows(m_.nv + 4 * ee.second->GetId(), 4)
                .middleCols(m_.nv + 3 * ee.second->GetId(), 3)
            << sqrt(2),
            0.0, -ee.second->mu(),
            -sqrt(2), 0.0, -ee.second->mu(),
            0.0, sqrt(2), -ee.second->mu(),
            0.0, -sqrt(2), -ee.second->mu();
        qp_data_->ubA.middleRows(m_.nv + 4 * ee.second->GetId(), 4).setConstant(0.0);
        qp_data_->lbA.middleRows(m_.nv + 4 * ee.second->GetId(), 4).setConstant(-qpOASES::INFTY);
    }

    LOG(INFO) << "acceleration bounds";
    for (int i = 0; i < m_.nv; i++) {
        qp_data_->ubx[i] = m_.qacc_max[i];
        qp_data_->lbx[i] = -m_.qacc_max[i];
    }

    LOG(INFO) << "control bounds";
    for (int i = 0; i < m_.nu; i++) {
        qp_data_->ubx[start_idx_.ctrl + i] = m_.ctrl_max[i];
        qp_data_->lbx[start_idx_.ctrl + i] = -m_.ctrl_max[i];
    }

    osc_setup_ = true;
    LOG(INFO) << "finished";
}