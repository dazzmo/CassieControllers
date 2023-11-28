#ifndef CONTROLLERS_OSC_OPTIONS_HPP
#define CONTROLLERS_OSC_OPTIONS_HPP

#include <qpOASES.hpp>
#include "controllers/types.h"

namespace controller {
namespace osc {

/**
 * @brief Options for creation of an OSC instance
 *
 */
struct Options : public controller::Options {
    Options() : max_number_working_set_recalculations(500),
                qpoases_print_level(qpOASES::PrintLevel::PL_LOW) {}
                
    // Maximum allowable recalculations of the working set when solving through qpOASES
    int max_number_working_set_recalculations;
    qpOASES::PrintLevel qpoases_print_level;
};

}  // namespace osc
}  // namespace controller

#endif /* CONTROLLERS_OSC_OPTIONS_HPP */
