#ifndef INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP
#define INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP

#include "controllers/types.h"

namespace controller {

class OutputPrescale {
   public:
    enum class RampType {
        NONE,
        RAMP_UP,
        RAMP_DOWN
    };

    void StartRamp(Scalar time, Scalar tau, const RampType &type);
    Scalar ApplyPrescale(Scalar time);

   private:
    Scalar t_ = 1.0;
    Scalar ramp_tau_ = 1.0;
    Scalar t_ramp_start_ = 0.0;

    RampType ramp_type_;

    Scalar RampUp(Scalar time, Scalar time_start);
    Scalar RampDown(Scalar time, Scalar time_start);
};

}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP */
