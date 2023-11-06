#include "controllers/output_prescale.h"

using namespace controller;

void OutputPrescale::StartRamp(Scalar time, Scalar tau, const RampType &type) {
    ramp_type_ = type;
    t_ramp_start_ = time;
}

Scalar OutputPrescale::RampUp(Scalar time, Scalar time_start) {
    return 1.0 - exp(-ramp_tau_ * (time - time_start));
}

Scalar OutputPrescale::RampDown(Scalar time, Scalar time_start) {
    return exp(-ramp_tau_ * (time - time_start));
}

/**
 * @brief Returns a prescale factor for the output torque of the system,
 * applicable if the system needs to ramp up/down the values upon initialisation or
 * where a soft-stop is required.
 *
 * @return Scalar
 */
Scalar OutputPrescale::ApplyPrescale(Scalar time) {
    Scalar prescale_factor = 1.0;
    switch (ramp_type_) {
        case RampType::NONE: {
            prescale_factor = 1.0;
            break;
        }
        case RampType::RAMP_UP: {
            prescale_factor = RampUp(time, t_ramp_start_);
            break;
        }
        case RampType::RAMP_DOWN: {
            prescale_factor = RampDown(time, t_ramp_start_);
            break;
        }
    }

    return prescale_factor;
}