#ifndef CONTROLLERS_OSC_OPTIONS_HPP
#define CONTROLLERS_OSC_OPTIONS_HPP

namespace controller {
namespace osc {

/**
 * @brief Options for creation of an OSC instance
 *
 */
struct Options {
    Options() : include_constraint_forces(true){};

    bool include_constraint_forces;
};

}  // namespace osc
}  // namespace controller

#endif /* CONTROLLERS_OSC_OPTIONS_HPP */
