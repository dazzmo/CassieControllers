#include "OutputPrescales/output_prescale.h"

void OutputPrescale::StartTorqueRampUp(double tau) {
    t_ramp_start_ = t_;
    ramp_tau_ = tau;
    u_prescale_type_ = TorquePreScale::RAMP_UP;
}

void OutputPrescale::StartTorqueRampDown(double tau) {
    t_ramp_start_ = t_;
    ramp_tau_ = tau;
    u_prescale_type_ = TorquePreScale::RAMP_DOWN;
}

double OutputPrescale::RampUp(void) {
    return 1.0 - exp(-ramp_tau_ * (t_ - t_ramp_start_));
}

double OutputPrescale::RampDown(void) {
    return exp(-ramp_tau_ * (t_ - t_ramp_start_));
}

/**
 * @brief Returns a prescale factor for the output torque of the system,
 * applicable if the system needs to ramp up/down the values upon initialisation or
 * where a soft-stop is required.
 *
 * @return double
 */
double OutputPrescale::ApplyTorquePreScale(void) {
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