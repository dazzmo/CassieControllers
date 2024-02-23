#include "controllers/osc/osc.h"

using namespace controller::osc;

OperationalSpaceController::OperationalSpaceController(const Model& model,
                                                       const Options& opt)
    : m_(const_cast<Model&>(model)),
      opt_(const_cast<Options&>(opt)),
      Controller(model.size().nu) {
    x_ = nullptr;
    c_ = nullptr;
    qp_data_ = nullptr;
    qp_ = nullptr;

    // Initialise weights on control outputs
    Wu_.resize(m_.size().nu);
    SetControlWeighting(m_.GetControlWeighting());
}

/**
 *
 * @brief Sets up the Operational Space Controller (OSC) program by initialising
 * the necessary matrices and vectors for computation.
 *
 * Quadratic programs created have the following variables for optimisation
 * If holonomic constraints are used explicitly (set by the options),
 * then we have x = [qacc, lambda_c, lambda_h, u]
 * If holonomic constraints are solved for implicitly,
 * then we have x = [qacc, lambda_c, u]
 *
 * @param opt
 */
void OperationalSpaceController::Init() {
    // If problem has changed, delete existing data
    if (is_initialised_) {
        delete x_;
        delete c_;
        delete qp_data_;
        qp_.reset();
    }

    // Create optimisation vector
    x_ = new OptimisationVector(m_.size(), m_.GetNumberOfContacts(),
                                m_.GetNumberOfHolonomicConstraints());

    // Create constraint vector
    c_ = new ConstraintVector(m_.size(), m_.GetNumberOfContacts(),
                              m_.GetNumberOfHolonomicConstraints());

    qp_data_ = new optimisation::QPOASESData(x_->sz, c_->sz);
    qp_ = std::unique_ptr<qpOASES::SQProblem>(
        new qpOASES::SQProblem(x_->sz, c_->sz));
    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
    qp_->setPrintLevel(opt_.qpoases_print_level);

    // Create projected constraint jacobian
    int ncp = m_.GetNumberOfProjectedConstraints();
    if (ncp > 0) {
        Jcp_ = Matrix::Zero(ncp, m_.size().nv);
        dJcpdq_v_ = Vector::Zero(ncp);
    }

    // Acceleration bounds
    // TODO: Use start index for generality
    for (int i = 0; i < m_.size().nv; ++i) {
        qp_data_->ubx[i] = m_.bounds().amax[i];
        qp_data_->lbx[i] = -m_.bounds().amax[i];
    }

    // End-effector force friction cones
    int id = 0;
    for (auto const& ee : m_.GetEndEffectorTaskMap()) {
        // Give each a unique contact id
        ee.second->SetId(id);
        // Set friction cone constraint
        qp_data_->A.middleRows(c_->friction_cones.start + 4 * id, 4)
                .middleCols(x_->lambda_c.start + 3 * id, ee.second->dim())
            << -sqrt(2), 0.0, ee.second->mu(), 
            sqrt(2), 0.0, ee.second->mu(), 
            0.0, -sqrt(2), ee.second->mu(), 
            0.0, sqrt(2), ee.second->mu();

        qp_data_->lbA
            .middleRows(c_->friction_cones.start + 4 * id, 4)
            .setZero();
        id++;
    }

    // Control bounds
    for (int i = 0; i < m_.size().nu; ++i) {
        qp_data_->ubx[x_->ctrl.start + i] = m_.bounds().umax[i];
        qp_data_->lbx[x_->ctrl.start + i] = -m_.bounds().umax[i];
    }

    // Indicate initialisation has been performed
    is_initialised_ = true;
}

/**
 * @brief Solves the current OSC program with the current values for the state
 * of the system at time t
 *
 * @param time
 * @param q
 * @param v
 */
void OperationalSpaceController::UpdateControl(Scalar time,
                                               const ConfigurationVector& q,
                                               const TangentVector& v) {
    if (is_initialised_ == false) {
        throw std::runtime_error(
            "OSC has not been initialised! Call Init() after adding all "
            "tasks.");
    }

    // Update model dynamics
    m_.UpdateModel(q, v);

    // Update references
    m_.UpdateReferences(time, q, v);

    // Number of projected constraints in problem
    Dimension ncp = m_.GetNumberOfProjectedConstraints();

    // Number of holonomic constraints in problem
    Dimension nch = m_.GetNumberOfHolonomicConstraints();

    // Construct projected constraints Jacobian and dJdt_v
    if (ncp > 0) {
        for (auto const& c : m_.GetProjectedConstraintMap()) {
            // Evaluate constraints from model
            c.second->Update(q, v);
            // Add to constraint jacobian
            Jcp_.middleRows(c.second->start(), c.second->dim())
                << c.second->J();
            dJcpdq_v_.middleRows(c.second->start(), c.second->dim())
                << c.second->dJdt_v();
        }
    }

    // Construct holonomic constraints Jacobian and dJdt_v
    if (nch > 0) {
        for (auto const& c : m_.GetHolonomicConstraintMap()) {
            // Evaluate constraints from model
            c.second->Update(q, v);
            // Add to constraints (J qacc + dJdt_v)
            qp_data_->A.bottomRows(nch)
                .middleRows(c.second->start(), c.second->dim())
                .middleCols(x_->qacc.start, x_->qacc.sz) = c.second->J();

            qp_data_->ubA.bottomRows(nch).middleRows(
                c.second->start(), c.second->dim()) = -c.second->dJdt_v();
            qp_data_->lbA.bottomRows(nch).middleRows(
                c.second->start(), c.second->dim()) = -c.second->dJdt_v();
        }
    }

    // ==== Dynamics constraints ====
    // Mass matrix
    // lbA <= A x <= ubA
    // M qacc + h = B u
    qp_data_->A.middleRows(c_->dynamics.start, c_->dynamics.sz)
        .middleCols(x_->qacc.start, x_->qacc.sz) = m_.dynamics().M;

    if (ncp > 0) {  // Projected constraints

        // Compute LDLT decomposition of M
        Eigen::LDLT<Eigen::MatrixXd> Mldlt = m_.dynamics().M.ldlt();

        // Inverse of the mass matrix and pseudo-inv of projector
        Eigen::MatrixXd Minv =
            Mldlt.solve(Eigen::MatrixXd::Identity(m_.size().nv, m_.size().nv));
        Matrix JMJT = Jcp_ * Minv * Jcp_.transpose();
        Matrix pinvJMJT =
            JMJT.completeOrthogonalDecomposition().pseudoInverse();

        // Compute null space projector
        N_ = Matrix::Identity(m_.size().nv, m_.size().nv) -
             Jcp_.transpose() * pinvJMJT * Jcp_ * Minv;

        // Equality bounds
        qp_data_->ubA.middleRows(c_->dynamics.start, c_->dynamics.sz) =
            -N_ * m_.dynamics().h - Jcp_.transpose() * pinvJMJT * dJcpdq_v_;
        qp_data_->lbA.middleRows(c_->dynamics.start, c_->dynamics.sz) =
            qp_data_->ubA.middleRows(c_->dynamics.start, c_->dynamics.sz);

    } else {  // Holonomic constraints / no constraints
        N_ = Matrix::Identity(m_.size().nv, m_.size().nv);

        // Equality bounds
        qp_data_->ubA.middleRows(c_->dynamics.start, c_->dynamics.sz) =
            -N_ * m_.dynamics().h;
        qp_data_->lbA.middleRows(c_->dynamics.start, c_->dynamics.sz) =
            qp_data_->ubA.middleRows(c_->dynamics.start, c_->dynamics.sz);
    }

    // End effector forces
    for (auto const& ee : m_.GetEndEffectorTaskMap()) {
        qp_data_->A.middleRows(c_->dynamics.start, c_->dynamics.sz)
            .middleCols(x_->lambda_c.start + 3 * ee.second->GetId(),
                        ee.second->dim()) = -N_ * ee.second->J().transpose();
    }

    // Holonomic constraint forces
    for (auto const& c : m_.GetHolonomicConstraintMap()) {
        qp_data_->A.middleRows(c_->dynamics.start, c_->dynamics.sz)
            .middleCols(x_->lambda_h.start + c.second->start(),
                        c.second->dim()) = -N_ * c.second->J().transpose();
    }

    // Actuation
    qp_data_->A.middleRows(c_->dynamics.start, c_->dynamics.sz)
        .middleCols(x_->ctrl.start, x_->ctrl.sz) = -N_ * m_.dynamics().B;

    // Reset cost
    qp_data_->H.setZero();
    qp_data_->g.setZero();

    // ==== Task cost addition ==== //
    for (auto const& task : m_.GetTaskMap()) {
        AddTaskCost(*qp_data_, task.second, q, v);
    }

    // ==== End-effector task cost addition ==== //
    for (auto const& task : m_.GetEndEffectorTaskMap()) {
        // Add cost for task
        AddTaskCost(*qp_data_, task.second, q, v);

        // Modify for contact
        Index idx = x_->lambda_c.start + 3 * task.second->GetId();
        if (task.second->inContact) {
            qp_data_->ubx.middleRows(idx, 3) << qpOASES::INFTY,
                qpOASES::INFTY, qpOASES::INFTY;
            qp_data_->lbx.middleRows(idx, 3) << -qpOASES::INFTY,
                -qpOASES::INFTY, qpOASES::ZERO;
        } else {
            qp_data_->ubx.middleRows(idx, 3) << qpOASES::ZERO, qpOASES::ZERO,
                qpOASES::ZERO;
            qp_data_->lbx.middleRows(idx, 3) << -qpOASES::ZERO,
                -qpOASES::ZERO, -qpOASES::ZERO;
        }
    }

    // ==== Torque regularisation cost addition ==== //
    for (int i = 0; i < m_.size().nu; ++i) {
        Index idx = x_->ctrl.start + i;
        qp_data_->H(idx, idx) += Wu_.diagonal()[i];
    }

    // ==== Solving ==== //

    // Set maximum number of working set recalculations
    int nWSR = opt_.max_number_working_set_recalculations;

    // Pre-multiply H by 2 to account for the expected form in qpOASES as 0.5 *
    // x^T * H * x
    qp_data_->H *= 2.0;

    // qpOASES return status
    qpOASES::returnValue status = qpOASES::SUCCESSFUL_RETURN;

    if (!hot_start_) {
        hot_start_ = true;
        status = qp_->init(qp_data_->H.data(), qp_data_->g.data(),
                           qp_data_->A.data(), qp_data_->lbx.data(),
                           qp_data_->ubx.data(), qp_data_->lbA.data(),
                           qp_data_->ubA.data(), nWSR);
    } else {
        status = qp_->hotstart(qp_data_->H.data(), qp_data_->g.data(),
                               qp_data_->A.data(), qp_data_->lbx.data(),
                               qp_data_->ubx.data(), qp_data_->lbA.data(),
                               qp_data_->ubA.data(), nWSR);
    }

    if (status != qpOASES::returnValue::SUCCESSFUL_RETURN) {
        LOG(ERROR) << "Failed";
        // TODO: See about handling this in a proper and sophisticated way
        int nWSR = opt_.max_number_working_set_recalculations;
        status = qp_->init(qp_data_->H.data(), qp_data_->g.data(),
                           qp_data_->A.data(), qp_data_->lbx.data(),
                           qp_data_->ubx.data(), qp_data_->lbA.data(),
                           qp_data_->ubA.data(), nWSR);
        // throw std::runtime_error("OSC has failed to solve current QP");
    }

    // Get solution and data
    qp_->getPrimalSolution(qp_data_->x.data());
    x_->Extract(qp_data_->x);
    qp_data_->cost = qp_->getObjVal();

    // Ramp torque up/down if required
    u_ = ApplyPrescale(time) * x_->ctrl.vec;
}

/**
 * @brief Adds cost to OSC program given a particular task
 *
 * @param qp_data
 * @param task
 * @param q
 * @param v
 */
void OperationalSpaceController::AddTaskCost(
    controller::optimisation::QPOASESData& qp_data,
    const std::shared_ptr<Task>& task, const ConfigurationVector& q,
    const TangentVector& v) {
    // Update task information
    task->Update(q, v);

    // Task weighting matrix
    const DiagonalMatrix& W = task->TaskWeightMatrix();

    // Task jacobian in qacc
    const Matrix& A = task->J();

    // Task constant vector (note: ErrorOutputPD = -Kp*e - Kd*edot)
    Vector a = task->ddr() - task->dJdt_v() + task->ErrorOutputPD();

    // Add to objective 0.5 * x^t H x + g^T x + c
    // cost = (A qacc - a)^T W (A qacc - a)
    qp_data.H.block(x_->qacc.start, x_->qacc.start, x_->qacc.sz, x_->qacc.sz) +=
        A.transpose() * W * A;
    qp_data.g.middleRows(x_->qacc.start, x_->qacc.sz) -=
        2.0 * A.transpose() * W * a;
}