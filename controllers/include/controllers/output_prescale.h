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

    void StartRamp(const RampType &type);

   private:
    Scalar ramp_tau_ = 1.0;
    Scalar t_ramp_start_ = 0.0;

    RampType ramp_type_;

    Scalar RampUp(void);
    Scalar RampDown(void);
};

}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_OUTPUT_PRESCALE_HPP */
