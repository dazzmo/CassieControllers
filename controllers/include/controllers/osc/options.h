#ifndef CONTROLLERS_OSC_OPTIONS_HPP
#define CONTROLLERS_OSC_OPTIONS_HPP

namespace controller {
namespace osc {

/**
 * @brief Options for creation of an OSC instance
 *
 */
struct Options {
    Options() : use_nulconstraint_lspace_projector(true){};

    bool use_constraint_nullspace_projector;
};

}  // namespace osc
}  // namespace controller

#endif /* CONTROLLERS_OSC_OPTIONS_HPP */
