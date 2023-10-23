#include "controllers/controller.h"

Controller::Controller(int nq, int nv, int nu) {
    LOG(INFO) << "Controller constructor";

    nq_ = nq;
    nv_ = nv;
    nu_ = nu;
}

void Controller::Resize(int nq, int nv, int nu) {
    qpos_ = Eigen::VectorXd::Zero(nq);
    qvel_ = Eigen::VectorXd::Zero(nv);
    qacc_ = Eigen::VectorXd::Zero(nv);
    u_ = Eigen::VectorXd::Zero(nu);

    qpos_bl_ = Eigen::VectorXd::Zero(nq);
    qpos_bu_ = Eigen::VectorXd::Zero(nq);

    qpos_0_ = Eigen::VectorXd::Zero(nq);

    qvel_max_ = 1e2 * Eigen::VectorXd::Ones(nv);
    qacc_max_ = 1e20 * Eigen::VectorXd::Ones(nv);
    u_max_ = Eigen::VectorXd::Zero(nu);
}

ControllerStatus Controller::Init() {
    // Starting time
    SetCurrentTime(0.0);

    // Set up matrices
    Resize(nq_, nv_, nu_);

    // Setup program (user-defined)
    SetupController();

    return ControllerStatus::SUCCESS;
}

void Controller::Update(double t) {
    SetCurrentTime(t);
    UpdateControl();
}

void Controller::StartTorqueRampUp(double tau) {
    t_ramp_start_ = t_;
    ramp_tau_ = tau;
    u_prescale_type_ = TorquePreScale::RAMP_UP;
}

void Controller::StartTorqueRampDown(double tau) {
    t_ramp_start_ = t_;
    ramp_tau_ = tau;
    u_prescale_type_ = TorquePreScale::RAMP_DOWN;
}

double Controller::RampUp(void) {
    return 1.0 - exp(-ramp_tau_ * (t_ - t_ramp_start_));
}

double Controller::RampDown(void) {
    return exp(-ramp_tau_ * (t_ - t_ramp_start_));
}

/**
 * @brief Returns a prescale factor for the output torque of the system,
 * applicable if the system needs to ramp up/down the values upon initialisation or
 * where a soft-stop is required.
 *
 * @return double
 */
double Controller::ApplyTorquePreScale(void) {
    double prescale_factor = 1.0;
    switch (u_prescale_type_) {
        case TorquePreScale::UNITY: {
            prescale_factor = 1.0;
            break;
        }
        case TorquePreScale::RAMP_UP: {
            prescale_factor = RampUp();
            break;
        }
        case TorquePreScale::RAMP_DOWN: {
            prescale_factor = RampDown();
            break;
        }
    }

    return prescale_factor;
}