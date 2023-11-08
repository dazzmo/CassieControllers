#ifndef CONTROLLERS_OSC_OPTIONS_HPP
#define CONTROLLERS_OSC_OPTIONS_HPP

#include "controllers/types.h"

namespace controller {
namespace osc {

/**
 * @brief Options for creation of an OSC instance
 *
 */
struct Options {
    Options() : include_constraint_forces(true),
                max_number_working_set_recalculations(500){

                };

    bool include_constraint_forces;
    int max_number_working_set_recalculations;
};

}  // namespace osc
}  // namespace controller

#endif /* CONTROLLERS_OSC_OPTIONS_HPP */
