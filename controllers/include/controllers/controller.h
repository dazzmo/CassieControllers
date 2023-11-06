#ifndef INCLUDE_CONTROLLERS_CONTROLLER_HPP
#define INCLUDE_CONTROLLERS_CONTROLLER_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <iostream>

#include "controllers/data.h"
#include "controllers/types.h"

namespace controller {

class Controller {
   public:
    Controller(Dimension nu);
    ~Controller() = default;

    void SetCurrentTime(double t) { d_.time = t; }
    const double& CurrentTime() const { return d_.time; }

    void SetControlFrequency(double freq) {
        assert(freq >= 0 && "Input frequency is less than zero");
        d_.freq = freq;
    }

    virtual void UpdateControl(Scalar time, const ConfigurationVector& q, const TangentVector& v) = 0;

    const ActuationVector& ControlOutput() const { return u_; }

   protected:
    // Controller data
    ControllerData d_;
    // Control output
    ActuationVector u_;

   private:
};

}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_CONTROLLER_HPP */
