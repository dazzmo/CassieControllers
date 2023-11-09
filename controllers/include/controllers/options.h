#ifndef INCLUDE_CONTROLLERS_OPTIONS_HPP
#define INCLUDE_CONTROLLERS_OPTIONS_HPP

#include "controllers/types.h"
#include "eigen3/Eigen/Core"

namespace controller {

struct Options {
    // Controller frequency
    Scalar frequency;
};

}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_OPTIONS_HPP */
