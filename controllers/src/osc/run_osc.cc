#include "controllers/osc/osc.h"

using namespace controller::osc;

/**
 * @brief Solves the current OSC program with the current values for the state of the
 * system at time t
 *
 */
void OperationalSpaceController::UpdateControl(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
    if (!osc_setup_) {
        throw std::runtime_error("OSC has not been created! Call CreateOSC() after adding all tasks.");
    }

    // Constant component of the QP cost 0.5 x^T * H * x + g^T * x + c
    double cost_c = 0.0;

    LOG(INFO) << "update";

    // Update model dynamics
    m_.UpdateModel(q, v);
    // Update references
    m_.UpdateReferences(time, q, v);

    LOG(INFO) << "constraints";

    // Update constraints and insert into stacked Jacobian and Jdot_qdot
    for (auto const& c : m_.GetConstraintMap()) {
        // Evaluate constraints from model
        c.second->Update(q, v);
        // Add to constraint jacobian
        Jceq_.middleRows(c.second->start(), c.second->dim()) << c.second->J();
        dJceqdq_.middleRows(c.second->start(), c.second->dim()) << c.second->Jdot_qdot();
    }

    LOG(INFO) << "dynamics";

    if (opt_->include_constraint_forces) {
        // ==== Dynamics constraints ====
        // Inertia matrix
        qp_data_->A.middleRows(x_->qacc.start, x_->qacc.sz).middleCols(0, m_.size().nv) << m_.dynamics().M;
        // Friction constraint forces
        LOG(INFO) << "ee";
        for (auto const& ee : m_.GetEndEffectorTaskMap()) {
            qp_data_->A.middleRows(x_->qacc.start, x_->qacc.sz).middleCols(x_->lambda_c.start + 3 * ee.second->GetId(), 3) = -ee.second->J().transpose();
        }
        LOG(INFO) << "c";
        // Holonomic constraint forces
        for (auto const& c : m_.GetConstraintMap()) {
            qp_data_->A.middleRows(x_->qacc.start, x_->qacc.sz).middleCols(x_->lambda_h.start + c.second->start(), c.second->dim()) = -c.second->J().transpose();
        }
        // Actuation
        LOG(INFO) << "u";
        qp_data_->A.middleRows(x_->qacc.start, x_->qacc.sz).middleCols(x_->ctrl.start, m_.size().nu) = -m_.dynamics().B;
        // Equality bounds
        LOG(INFO) << "bounds";
        qp_data_->ubA.middleRows(x_->qacc.start, x_->qacc.sz) = -m_.dynamics().h;
        qp_data_->lbA.middleRows(x_->qacc.start, x_->qacc.sz) = -m_.dynamics().h;

        // ==== Holnomic equality constraints ====
        LOG(INFO) << "constraints";
        if (m_.GetNumberOfConstraints()) {
            qp_data_->A.bottomRows(m_.GetNumberOfConstraints()).middleCols(x_->lambda_h.start, m_.GetNumberOfConstraints()) << Jceq_;
            qp_data_->ubA.bottomRows(m_.GetNumberOfConstraints()) << -dJceqdq_;
            qp_data_->lbA.bottomRows(m_.GetNumberOfConstraints()) << -dJceqdq_;
        }

    } else {
        // ==== Projected dynamics constraints ====
        if (m_.GetNumberOfConstraints()) {
            Matrix invM = m_.dynamics().M;
            Matrix JMJT = Jceq_ * invM * Jceq_.transpose();
            Matrix pinvJMJT = JMJT.completeOrthogonalDecomposition().pseudoInverse();
            // Compute null space matrix
            N_ = Matrix::Identity(m_.size().nv, m_.size().nv) - Jceq_.transpose() * pinvJMJT * Jceq_ * invM;
            // Equality bounds
            qp_data_->ubA.topRows(m_.size().nv) = -N_ * m_.dynamics().h - Jceq_.transpose() * pinvJMJT * dJceqdq_;
            qp_data_->lbA.topRows(m_.size().nv) = qp_data_->ubA.topRows(m_.size().nv);
        } else {
            N_ = Matrix::Identity(m_.size().nv, m_.size().nv);
            // Equality bounds
            qp_data_->ubA.topRows(m_.size().nv) = -N_ * m_.dynamics().h;
            qp_data_->lbA.topRows(m_.size().nv) = qp_data_->ubA.topRows(m_.size().nv);
        }

        // End effector forces
        for (auto const& ee : m_.GetEndEffectorTaskMap()) {
            qp_data_->A.topRows(m_.size().nv).middleCols(x_->lambda_c.start + 3 * ee.second->GetId(), 3) = -N_ * ee.second->J().transpose();
        }
        // Actuation
        qp_data_->A.topRows(m_.size().nv).middleCols(x_->ctrl.start, x_->ctrl.sz) = -N_ * m_.dynamics().B;
    }

    // Reset cost
    qp_data_->H.setZero();
    qp_data_->g.setZero();

    LOG(INFO) << "regular tasks";
    for (auto const& task : m_.GetTaskMap()) {
        task.second->Update(q, v);
        double w = task.second->weight();
        // Task jacobian in qacc
        const Matrix& A = task.second->J();
        // Task constant vector
        Vector a = task.second->Jdot_qdot() + task.second->ErrorOutputPD();

        // Add to objective
        qp_data_->H.block(x_->qacc.start, x_->qacc.start, x_->qacc.sz, x_->qacc.sz) += w * A.transpose() * A;
        qp_data_->g.middleRows(x_->qacc.start, x_->qacc.sz) += 2.0 * w * A.transpose() * a;
        cost_c += w * a.dot(a);
    }

    LOG(INFO) << "end effector tasks";
    for (auto const& task : m_.GetEndEffectorTaskMap()) {
        // Update the task
        task.second->Update(q, v);
        double w = task.second->weight();
        // Task jacobian in qacc
        const Matrix& A = task.second->J();
        // Task constant vector
        Vector3 a = task.second->Jdot_qdot() + task.second->ErrorOutputPD();

        // Add to objective
        qp_data_->H.block(x_->qacc.start, x_->qacc.start, x_->qacc.sz, x_->qacc.sz) += w * A.transpose() * A;
        qp_data_->g.middleRows(x_->qacc.start, x_->qacc.sz) += 2.0 * w * A.transpose() * a;
        cost_c += w * a.dot(a);

        // Contact
        if (task.second->inContact) {
            qp_data_->ubx.middleRows(x_->lambda_c.start + task.second->start(), task.second->dim()) << qpOASES::INFTY, qpOASES::INFTY, qpOASES::INFTY;
            qp_data_->lbx.middleRows(x_->lambda_c.start + task.second->start(), task.second->dim()) << -qpOASES::INFTY, -qpOASES::INFTY, qpOASES::ZERO;
        } else {
            qp_data_->ubx.middleRows(x_->lambda_c.start + task.second->start(), task.second->dim()) << qpOASES::ZERO, qpOASES::ZERO, qpOASES::ZERO;
            qp_data_->lbx.middleRows(x_->lambda_c.start + task.second->start(), task.second->dim()) << -qpOASES::ZERO, -qpOASES::ZERO, -qpOASES::ZERO;
        }
    }

    // Torque regularisation
    for (int i = 0; i < m_.size().nu; ++i) {
        qp_data_->H(x_->ctrl.start + i, x_->ctrl.start + i) += torque_weight_;
    }

    LOG(INFO) << "solve";
    // Set maximum number of working set recalculations
    int nWSR = opt_->max_number_working_set_recalculations;
    // Pre-multiply H by 2 to account for the expected form in qpOASES as 0.5 * x^T * H * x
    qp_data_->H *= 2.0;
    // qpOASES return status
    qpOASES::returnValue status = qpOASES::SUCCESSFUL_RETURN;

    if (!hot_start_) {
        hot_start_ = true;
        status = qp_->init(qp_data_->H.data(), qp_data_->g.data(), qp_data_->A.data(),
                           qp_data_->lbx.data(), qp_data_->ubx.data(),
                           qp_data_->lbA.data(), qp_data_->ubA.data(),
                           nWSR);
    } else {
        status = qp_->hotstart(qp_data_->H.data(), qp_data_->g.data(), qp_data_->A.data(),
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

    // Extract solution components
    x_->qacc.vec = qp_data_->x.middleRows(x_->qacc.start, x_->qacc.sz);
    x_->lambda_c.vec = qp_data_->x.middleRows(x_->lambda_c.start, x_->lambda_c.sz);
    x_->ctrl.vec = qp_data_->x.middleRows(x_->ctrl.start, x_->ctrl.sz);
    if (opt_->include_constraint_forces) {
        x_->lambda_h.vec = qp_data_->x.middleRows(x_->lambda_h.start, x_->lambda_h.sz);
    }

    // Also add constraint forces to each task and constraint for convenience
    for (auto const& ee : m_.GetEndEffectorTaskMap()) {
        ee.second->lambda() = qp_data_->x.middleRows(x_->lambda_c.start + 3 * ee.second->GetId(), 3);
    }

    // Constraint forces (if included in the optimisation)
    if (opt_->include_constraint_forces) {
        for (auto const& c : m_.GetConstraintMap()) {
            c.second->lambda() = qp_data_->x.middleRows(c.second->start(), c.second->dim());
        }
    }

    // Set output signal for control
    LOG(INFO) << "u (solved): " << u_.transpose();
    u_ = x_->ctrl.vec;
}