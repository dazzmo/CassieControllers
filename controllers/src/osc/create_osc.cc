#include "controllers/osc/osc.h"

using namespace controller::osc;

OperationalSpaceController::OperationalSpaceController(const Model& model) : m_(const_cast<Model&>(model)),
                                                                             Controller(model.size().nu) {
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
void OperationalSpaceController::CreateOSC(const Options& opt) {
    LOG(INFO) << "CreateOSC";
    // Create options
    opt_ = new Options(opt);
    // Create program
    LOG(INFO) << "x";
    x_ = new OptimisationVector(m_.size(), m_.GetNumberOfContacts(), m_.GetNumberOfConstraints(), opt);
    LOG(INFO) << "c";
    c_ = new ConstraintVector(m_.size(), m_.GetNumberOfContacts(), m_.GetNumberOfConstraints(), opt);

    LOG(INFO) << "qp";
    qp_data_ = new QPData(x_->sz, c_->sz);
    qp_ = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(x_->sz, c_->sz));
    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);

    LOG(INFO) << "J";
    // Create holonomic constraint jacobian
    Jceq_ = Matrix::Zero(m_.GetNumberOfConstraints(), m_.size().nv);
    dJceqdq_ = Vector::Zero(m_.GetNumberOfConstraints());

    LOG(INFO) << "friction cone approximations";
    for (auto const& ee : m_.GetEndEffectorTaskMap()) {
        // Friction cone constraints
        // TODO: Will need to account for surface normals
        qp_data_->A.middleRows(c_->lambda_c.start + 4 * ee.second->GetId(), 4)
                .middleCols(x_->lambda_c.start + 3 * ee.second->GetId(), 3)
            << sqrt(2), 0.0, -ee.second->mu(),
            -sqrt(2), 0.0, -ee.second->mu(),
            0.0, sqrt(2), -ee.second->mu(),
            0.0, -sqrt(2), -ee.second->mu();
        qp_data_->ubA.middleRows(c_->lambda_c.start + 4 * ee.second->GetId(), 4).setConstant(0.0);
        qp_data_->lbA.middleRows(c_->lambda_c.start + 4 * ee.second->GetId(), 4).setConstant(-qpOASES::INFTY);
    }

    LOG(INFO) << "acceleration bounds";
    for (int i = 0; i < m_.size().nv; ++i) {
        qp_data_->ubx[i] = m_.bounds().amax[i];
        qp_data_->lbx[i] = -m_.bounds().amax[i];
    }

    LOG(INFO) << "control bounds";
    for (int i = 0; i < m_.size().nu; ++i) {
        qp_data_->ubx[x_->ctrl.start + i] = m_.bounds().umax[i];
        qp_data_->lbx[x_->ctrl.start + i] = -m_.bounds().umax[i];
    }

    osc_setup_ = true;
    LOG(INFO) << "finished";
}