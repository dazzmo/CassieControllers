#include "controllers/osc/osc.h"

using namespace controller::osc;

OperationalSpaceController::OperationalSpaceController(const Model& model) : m_(const_cast<Model&>(model)), Controller() {
    // Add tasks
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

    // Create program
    res_ = new OptimisationVector(m_.size(), m_.GetNumberOfContacts(), nceq_, opt);

    qp_ = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(qp_data_->nx, qp_data_->nce));
    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);

    // Create holonomic constraint jacobian
    Jceq_ = Matrix::Zero(nceq_, m_.size().nv);
    dJceqdq_ = Vector::Zero(nceq_);

    LOG(INFO) << "friction cone approximations";
    for (auto const& ee : m_.GetEndEffectorTaskMap()) {
        // Friction cone constraints
        // TODO: Will need to account for surface normals
        qp_data_->A.middleRows(m_.size().nv + 4 * ee.second->GetId(), 4)
                .middleCols(m_.size().nv + 3 * ee.second->GetId(), 3)
            << sqrt(2),
            0.0, -ee.second->mu(),
            -sqrt(2), 0.0, -ee.second->mu(),
            0.0, sqrt(2), -ee.second->mu(),
            0.0, -sqrt(2), -ee.second->mu();
        qp_data_->ubA.middleRows(m_.size().nv + 4 * ee.second->GetId(), 4).setConstant(0.0);
        qp_data_->lbA.middleRows(m_.size().nv + 4 * ee.second->GetId(), 4).setConstant(-qpOASES::INFTY);
    }

    LOG(INFO) << "acceleration bounds";
    for (int i = 0; i < m_.size().nv; ++i) {
        qp_data_->ubx[i] = m_.bounds().amax[i];
        qp_data_->lbx[i] = -m_.bounds().amax[i];
    }

    LOG(INFO) << "control bounds";
    for (int i = 0; i < m_.size().nu; ++i) {
        qp_data_->ubx[res_->ctrl.start + i] = m_.bounds().umax[i];
        qp_data_->lbx[res_->ctrl.start + i] = -m_.bounds().umax[i];
    }

    osc_setup_ = true;
    LOG(INFO) << "finished";
}