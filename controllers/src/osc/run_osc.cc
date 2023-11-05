#include "controllers/osc/osc.h"

using namespace controller::osc;

/**
 * @brief Solves the current OSC program with the current values for the state of the
 * system at time t
 *
 */
const ActuationVector& OperationalSpaceController::RunOSC() {
    if (!osc_setup_) {
        throw std::runtime_error("OSC has not been created! Call CreateOSC() after adding all tasks.");
    }

    double c = 0.0;

    LOG(INFO) << "constraints";
    int jac_idx = 0;

    for (auto const& c : constraints_) {
        // Evaluate constraints from model
        ModelData::Constraint &con = m_->GetConstraints(c.first);
        // Add to constraint jacobian
        Jh_.middleRows(jac_idx, con.n) << con.J()
        dJhdq_.middleRows(jac_idx, con.n) << con.dJdq();
        jac_idx += con.n;
    }

    // Update model dynamics
    m_->UpdateModel(m_->state().q, m_->state().v);

    if (opt_->use_constraint_nullspace_projector) {
        // Holnomic equality constraints
        qp_data_->A.bottomRows(m_.ng).middleCols(start_idx_.lambda_h, m_.ng) << Jh_;
        qp_data_->ubA.bottomRows(m_.ng) << -dJhdq_;
        qp_data_->lbA.bottomRows(m_.ng) << -dJhdq_;
        // Inertia matrix
        qp_data_->A.topRows(m_->size().nv).middleCols(0, m_->size().nv) << m_.M;
        // Friction constraint forces
        for (auto const& ee : ee_tasks_) {
            qp_data_->A.topRows(m_->size().nv).middleCols(start_idx_.lambda_c + 3 * ee.second->GetId(), 3) = -ee.second->J().transpose();
        }
        // Holonomic constraint forces
        int con_idx = 0;
        for (auto const& c : constraints_) {
            qp_data_->A.topRows(m_->size().nv).middleCols(start_idx_.lambda_h + con_idx, c.second->dim()) = -c.second->J().transpose();
            con_idx += c.second->dim();
        }
        // Actuation
        qp_data_->A.topRows(m_->size().nv).middleCols(start_idx_.ctrl, m_.nu) = -m_.B;

        // Dynamic constraints
        qp_data_->ubA.topRows(m_->size().nv) = -m_.h;
        qp_data_->lbA.topRows(m_->size().nv) = -m_.h;

    } else {
        Eigen::MatrixXd invM = m_.M.inverse();
        Eigen::MatrixXd JMJT = Jh_ * invM * Jh_.transpose();
        Eigen::MatrixXd pinvJMJT = JMJT.completeOrthogonalDecomposition().pseudoInverse();

        // Compute null space matrix
        N_ = Eigen::MatrixXd::Identity(m_->size().nv, m_->size().nv) - Jh_.transpose() * pinvJMJT * Jh_ * invM;

        // If using explicit or implicit
        for (auto const& ee : ee_tasks_) {
            qp_data_->A.topRows(m_->size().nv).middleCols(start_idx_.lambda_c + 3 * ee.second->GetId(), 3) = -N_ * ee.second->J().transpose();
        }
        // Actuation
        qp_data_->A.topRows(m_->size().nv).middleCols(start_idx_.ctrl, m_.nu) = -N_ * m_.B;
        // Dynamic constraints
        qp_data_->ubA.topRows(m_->size().nv) = -N_ * m_.h - Jh_.transpose() * pinvJMJT * dJhdq_;
        qp_data_->lbA.topRows(m_->size().nv) = qp_data_->ubA.topRows(m_->size().nv);
    }

    // Reset cost
    qp_data_->H.setZero();
    qp_data_->g.setZero();

    LOG(INFO) << "regular tasks";
    for (auto const& task : tasks_) {
        task.second->Update(m_->state().q, m_->state().v);
        double w = task.second->weight();
        // Task jacobian in qacc
        const Eigen::MatrixXd& A = task.second->J();
        // Task constant vector
        // Eigen::Vector3d a = task.second->dJdq() + task.second->();

        // Add to objective
        qp_data_->H.topLeftCorner(m_->size().nv, m_->size().nv) += w * A.transpose() * A;
        qp_data_->g.topRows(m_->size().nv) += 2.0 * w * A.transpose() * a;
        c += w * a.dot(a);
    }

    LOG(INFO) << "end effector tasks";
    for (auto const& task : ee_tasks_) {
        // Update the task
        task.second->Update(m_->state().q, m_->state().v);
        double w = task.second->weight();
        // Task jacobian in qacc
        const Eigen::MatrixXd& A = task.second->J();
        // Task constant vector
        Eigen::Vector3d a = task.second->dJdq() + task.second->TaskErrorPD();

        // Add to objective
        qp_data_->H.topLeftCorner(m_->size().nv, m_->size().nv) += w * A.transpose() * A;
        qp_data_->g.topRows(m_->size().nv) += 2.0 * w * A.transpose() * a;
        c += w * a.dot(a);

        // Contact
        if (task.second->inContact) {
            qp_data_->ubx.middleRows(m_->size().nv + 3 * task.second->GetId(), 3) << qpOASES::INFTY, qpOASES::INFTY, qpOASES::INFTY;
            qp_data_->lbx.middleRows(m_->size().nv + 3 * task.second->GetId(), 3) << -qpOASES::INFTY, -qpOASES::INFTY, qpOASES::ZERO;
        } else {
            qp_data_->ubx.middleRows(m_->size().nv + 3 * task.second->GetId(), 3) << qpOASES::ZERO, qpOASES::ZERO, qpOASES::ZERO;
            qp_data_->lbx.middleRows(m_->size().nv + 3 * task.second->GetId(), 3) << -qpOASES::ZERO, -qpOASES::ZERO, -qpOASES::ZERO;
        }
    }

    // Torque regularisation
    for (int i = 0; i < m_->size().nu; ++i) {
        qp_data_->H(start_idx_.ctrl + i, start_idx_.ctrl + i) += torque_weight_;
    }

    LOG(INFO) << "solve";
    int nWSR = 1000;

    qp_data_->H *= 2.0;
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
    res_->qacc = qp_data_->x.topRows(m_->size().nv).transpose();
    res_->lambda_c = qp_data_->x.middleRows(start_idx_.lambda_c, 3 * m_.nc);
    if (opt_->use_constraint_nullspace_projector) {
        res_->lambda_h = qp_data_->x.middleRows(start_idx_.lambda_h, m_.ng);
    }
    res_->ctrl = qp_data_->x.bottomRows(m_.nu);

    // Also add constraint forces to each task and constraint for convenience
    for (auto const& ee : ee_tasks_) {
        ee.second->lambda() = qp_data_->x.middleRows(m_->size().nv + 3 * ee.second->GetId(), 3);
    }
    // Constraint forces (if included in the optimisation)
    if (opt_->use_constraint_nullspace_projector) {
        int jac_idx = 0;
        for (auto const& c : constraints_) {
            c.second->lambda() = qp_data_->x.middleRows(start_idx_.lambda_h + jac_idx, c.second->dim());
            jac_idx += c.second->dim();
        }
    }

    return res_->ctrl;
}