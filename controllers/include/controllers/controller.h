#ifndef INCLUDE_CONTROLLERS_CONTROLLER_HPP
#define INCLUDE_CONTROLLERS_CONTROLLER_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <iostream>

#include "controllers/options.h"
#include "controllers/types.h"

namespace controller {

class Controller {
   public:
    Controller(Dimension nu, const Options& opt = Options());
    ~Controller() = default;

    void SetControlFrequency(Scalar freq) {
        assert(freq >= 0 && "Input frequency is less than zero");
        opt_.frequency = freq;
    }
    const Scalar ControlFrequency() const { return opt_.frequency; }

    virtual void UpdateControl(Scalar time, const ConfigurationVector& q, const TangentVector& v) = 0;

    const ActuationVector& ControlOutput() const { return u_; }

   protected:
    // Control output
    ActuationVector u_;

   private:
    // Controller options
    Options opt_;
};

}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_CONTROLLER_HPP */
