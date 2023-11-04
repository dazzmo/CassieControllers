#ifndef INCLUDE_CONTROLLERS_CONTROLLER_HPP
#define INCLUDE_CONTROLLERS_CONTROLLER_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <iostream>

#include "controllers/data.h"
#include "controllers/model.h"
#include "controllers/types.h"

namespace controller {

class Controller {
   public:
    Controller(const DynamicModel& model);
    ~Controller() = default;

    void SetCurrentTime(double t) { d_.time = t; }
    const double& CurrentTime() const { return d_.time; }

    void SetControlFrequency(double freq) {
        assert(freq >= 0 && "Input frequency is less than zero");
        d_.freq = freq;
    }

    virtual void UpdateControl(Scalar time, const ConfigurationVector& qpos, const TangentVector& qvel) = 0;

   protected:
    // Model data for controller
    DynamicModel* m_;
    // Controller data
    ControllerData d_;

   private:
};

}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_CONTROLLER_HPP */
