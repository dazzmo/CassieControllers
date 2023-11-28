#include "controllers/osc/osc.h"

using namespace controller::osc;

OperationalSpaceController::OperationalSpaceController(const Model& model, const Options& opt) : m_(const_cast<Model&>(model)),
                                                                                                 opt_(const_cast<Options&>(opt)),
                                                                                                 Controller(model.size().nu) {
    x_ = nullptr;
    c_ = nullptr;
    qp_data_ = nullptr;
    qp_ = nullptr;

    // Initialise any weights
    Wu_.resize(m_.size().nu);
    Wu_.diagonal().setZero();
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
    x_ = new OptimisationVector(m_.size(), m_.GetNumberOfContacts(), m_.GetNumberOfHolonomicConstraints());

    // Create constraint vector
    c_ = new ConstraintVector(m_.size(), m_.GetNumberOfContacts(), m_.GetNumberOfHolonomicConstraints());

    qp_data_ = new optimisation::QPOASESData(x_->sz, c_->sz);
    qp_ = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(x_->sz, c_->sz));
    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
    qp_->setPrintLevel(opt_.qpoases_print_level);

    // Create projected constraint jacobian
    int ncp = m_.GetNumberOfProjectedConstraints();
    if (ncp > 0) {
        Jcp_ = Matrix::Zero(ncp, m_.size().nv);
        dJcpdq_v_ = Vector::Zero(ncp);
    }

    // Acceleration bounds
    for (int i = 0; i < m_.size().nv; ++i) {
        qp_data_->ubx[i] = m_.bounds().amax[i];
        qp_data_->lbx[i] = -m_.bounds().amax[i];
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
 * @brief Solves the current OSC program with the current values for the state of the
 * system at time t
 *
 * @param time
 * @param q
 * @param v
 */
void OperationalSpaceController::UpdateControl(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
    if (is_initialised_ == false) {
        throw std::runtime_error("OSC has not been initialised! Call Init() after adding all tasks.");
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
            Jcp_.middleRows(c.second->start(), c.second->dim()) << c.second->J();
            dJcpdq_v_.middleRows(c.second->start(), c.second->dim()) << c.second->dJdt_v();
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

            qp_data_->ubA.bottomRows(nch).middleRows(c.second->start(), c.second->dim()) = -c.second->dJdt_v(); // TODO: Should upper bound be positive? Or is it a >= thing?
            qp_data_->lbA.bottomRows(nch).middleRows(c.second->start(), c.second->dim()) = -c.second->dJdt_v();
        }
    }

    // ==== Dynamics constraints ====
    // Mass matrix
    qp_data_->A.middleRows(x_->qacc.start, x_->qacc.sz)
        .middleCols(x_->qacc.start, x_->qacc.sz) = m_.dynamics().M; // TODO: What does this do?

    if (ncp > 0) {
        Matrix invM = m_.dynamics().M; // TODO: Shouldn't this be the INVERSE of the mass matrix? (But how to do efficiently?)
        Matrix JMJT = Jcp_ * invM * Jcp_.transpose();
        Matrix pinvJMJT = JMJT.completeOrthogonalDecomposition().pseudoInverse();

        // Compute null space matrix
        N_ = Matrix::Identity(m_.size().nv, m_.size().nv) - Jcp_.transpose() * pinvJMJT * Jcp_ * invM;

        // Equality bounds
        qp_data_->ubA.topRows(m_.size().nv) = -N_ * m_.dynamics().h - Jcp_.transpose() * pinvJMJT * dJcpdq_v_; // TODO: What about other forces here?
        qp_data_->lbA.topRows(m_.size().nv) = qp_data_->ubA.topRows(m_.size().nv);

    } else {
        N_ = Matrix::Identity(m_.size().nv, m_.size().nv);

        // Equality bounds
        qp_data_->ubA.topRows(m_.size().nv) = -N_ * m_.dynamics().h;
        qp_data_->lbA.topRows(m_.size().nv) = qp_data_->ubA.topRows(m_.size().nv);
    }

    // End effector forces
    for (auto const& ee : m_.GetEndEffectorTaskMap()) {
        qp_data_->A.topRows(m_.size().nv).middleCols(x_->lambda_c.start + ee.second->start(), ee.second->dim()) = -N_ * ee.second->J().transpose();
    }

    // Holonomic constraint forces
    for (auto const& c : m_.GetHolonomicConstraintMap()) {
        qp_data_->A.topRows(m_.size().nv).middleCols(x_->lambda_h.start + c.second->start(), c.second->dim()) = -N_ * c.second->J().transpose();
    }

    // Actuation
    qp_data_->A.topRows(m_.size().nv).middleCols(x_->ctrl.start, x_->ctrl.sz) = -N_ * m_.dynamics().B;

    // Reset cost
    qp_data_->H.setZero();
    qp_data_->g.setZero();

    // ==== Task cost addition ==== //
    for (auto const& task : m_.GetTaskMap()) {
        task.second->Update(q, v);

        // Task weighting matrix
        const DiagonalMatrix& W = task.second->TaskWeightMatrix();
        
        // Task jacobian in qacc
        const Matrix& A = task.second->J();
        
        // Task constant vector (note: ErrorOutputPD = -Kp*e - Kd*edot)
        Vector a = task.second->ddr() - task.second->dJdt_v() + task.second->ErrorOutputPD();

        // Add to objective 0.5 * x^t H x + g^T x + c
        // cost = (A qacc - a)^T W (A qacc - a)
        qp_data_->H.block(x_->qacc.start, x_->qacc.start, x_->qacc.sz, x_->qacc.sz) += A.transpose() * W * A;
        qp_data_->g.middleRows(x_->qacc.start, x_->qacc.sz) -= 2.0 * A.transpose() * W * a;
        qp_data_->cost_const += (W * a).dot(a);
    }

    // ==== End-effector task cost addition ==== //
    for (auto const& task : m_.GetEndEffectorTaskMap()) {
        
        // Update the task
        task.second->Update(q, v);
        
        // Task weighting matrix
        const DiagonalMatrix& W = task.second->TaskWeightMatrix();
        
        // Task jacobian in qacc
        const Matrix& A = task.second->J();
        
        // Task constant vector (note: ErrorOutputPD = -Kp*e - Kd*edot)
        Vector3 a = task.second->ddr() - task.second->dJdt_v() + task.second->ErrorOutputPD();

        // Add to objective (|| A qacc - a ||^2)
        qp_data_->H.block(x_->qacc.start, x_->qacc.start, x_->qacc.sz, x_->qacc.sz) += A.transpose() * W * A;
        qp_data_->g.middleRows(x_->qacc.start, x_->qacc.sz) -= 2.0 * A.transpose() * W * a;
        qp_data_->cost_const += (W * a).dot(a);

        // Contact
        Dimension dim = task.second->dim();
        Index idx = x_->lambda_c.start + task.second->start();
        if (task.second->inContact) {
            qp_data_->ubx.middleRows(idx, dim) << qpOASES::INFTY, qpOASES::INFTY, qpOASES::INFTY;
            qp_data_->lbx.middleRows(idx, dim) << -qpOASES::INFTY, -qpOASES::INFTY, qpOASES::ZERO;
        } else {
            qp_data_->ubx.middleRows(idx, dim) << qpOASES::ZERO, qpOASES::ZERO, qpOASES::ZERO;
            qp_data_->lbx.middleRows(idx, dim) << -qpOASES::ZERO, -qpOASES::ZERO, -qpOASES::ZERO;
        }
    }

    // ==== Torque regularisation cost addition ==== //
    for (int i = 0; i < m_.size().nu; ++i) {
        Index idx = x_->ctrl.start + i;
        qp_data_->H(idx, idx) += Wu_.diagonal()[i]; // TODO: Double-check
    }

    // ==== Solving ==== //

    // Set maximum number of working set recalculations
    int nWSR = opt_.max_number_working_set_recalculations;
    
    // Pre-multiply H by 2 to account for the expected form in qpOASES as 0.5 * x^T * H * x
    qp_data_->H *= 2.0;

    // qpOASES return status
    qpOASES::returnValue status = qpOASES::SUCCESSFUL_RETURN;

    if (!hot_start_) {
        hot_start_ = true;
        status = qp_->init(qp_data_->H.data(), qp_data_->g.data(),
                           qp_data_->A.data(),
                           qp_data_->lbx.data(), qp_data_->ubx.data(),
                           qp_data_->lbA.data(), qp_data_->ubA.data(),
                           nWSR);
    } else {
        status = qp_->hotstart(qp_data_->H.data(), qp_data_->g.data(),
                               qp_data_->A.data(),
                               qp_data_->lbx.data(), qp_data_->ubx.data(),
                               qp_data_->lbA.data(), qp_data_->ubA.data(),
                               nWSR);
    }

    if (status != qpOASES::returnValue::SUCCESSFUL_RETURN) {
        LOG(ERROR) << "Failed";
        throw std::runtime_error("OSC has failed to solve current QP");
    }

    // Get solution
    qp_->getPrimalSolution(qp_data_->x.data());
    // qp_data_->cost_const += qp_->get // TODO: Is this needed?

    // Extract solution components
    x_->Extract(qp_data_->x);
    c_->Extract(qp_data_->x); // TODO: Should this be c not x?

    u_ = ApplyPrescale(time) * x_->ctrl.vec; // TODO: Do we actually need the pre-scale? Might want to remove it, but not doing any harm at the moment
}