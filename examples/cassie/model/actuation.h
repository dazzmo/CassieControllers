#ifndef MODEL_ACTUATION
#define MODEL_ACTUATION

#include <damotion/utils/eigen_wrapper.h>

#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

/**
 * @brief Computes the closed-loop constraints associated with the four-bar
 * linkage on each leg of the Cassie robot
 *
 * @param model Pinocchio model of a cassie robot
 * @param data Pinocchio data associated with the cassie model
 * @return casadi::SX Constraint vector for closed-loop constraint of both legs
 */
casadi::SX CassieActuationMatrix(pinocchio::ModelTpl<casadi::SX> &model,
                                 pinocchio::DataTpl<casadi::SX> &data,
                                 const casadi::SX &qpos, const casadi::SX &qvel);

#endif /* MODEL_ACTUATION */

// Order: hip roll, yaw, pitch, knee, shin (spring), tarsus, heel
// spring, toe
// initial_state().q << 0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267,
// 0.0,
//     -1.5968, -0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0,
//     -1.5968;

// Add bounds (from cassie.xml, different from kinematic model on
// wiki)
// bounds().qmin << -15.0, -22.5, -50.0, -164.0, -20.0, 50.0, -20.0,
//     -140.0, -15.0, -22.5, -50.0, -164.0, -20.0, 50.0, -20.0,
//     -140.0;
// bounds().qmax << 22.5, 22.5, 80.0, -37.0, 20.0, 170.0, 20.0,
// -30.0,
//     22.5, 22.5, 80.0, -37.0, 20.0, 170.0, 20.0, -30.0;
// bounds().qmin *= M_PI / 180;
// bounds().qmax *= M_PI / 180;
// bounds().vmax << 12.15, 12.15, 8.5, 8.5, 20, 20,
// 20, 11.52, 12.15, 12.15, 8.5, 8.5, 20, 20, 20, 11.52;