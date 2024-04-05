#ifndef MODEL_ACTUATION_H
#define MODEL_ACTUATION_H

#include <damotion/utils/eigen_wrapper.h>

#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

/**
 * @brief Computes the closed-loop constraints associated with the four-bar
 * linkage on each leg of the 3DOFArm robot
 *
 * @param model Pinocchio model of a 3DOFArm robot
 * @param data Pinocchio data associated with the 3DOFArm model
 * @return casadi::SX Constraint vector for closed-loop constraint of both legs
 */
casadi::SX ArmActuationMatrix(pinocchio::ModelTpl<casadi::SX> &model,
                                  pinocchio::DataTpl<casadi::SX> &data,
                                  casadi::SX &qpos, casadi::SX &qvel) {
    typedef casadi::SX Scalar;

    // Add dynamics for 3DOFArm
    casadi::SX B(model.nv, 3);
    // TODO - Could add damping and friction effects here
    // Left Motors
    B(model.joints[model.getJointId("shoulder")].idx_v(), 0) = 1.0;
    B(model.joints[model.getJointId("elbow")].idx_v(), 1) = 1.0;
    B(model.joints[model.getJointId("wrist")].idx_v(), 2) = 1.0;

    // Return actuation matrix
    return B;
}

// Order: hip roll, yaw, pitch, knee, shin (spring), tarsus, heel
// spring, toe
// initial_state().q << 0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267,
// 0.0,
//     -1.5968, -0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0,
//     -1.5968;

// Add bounds (from 3DOFArm.xml, different from kinematic model on
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

#endif /* MODEL_ACTUATION_H */
