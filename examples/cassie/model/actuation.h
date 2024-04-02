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
                                 casadi::SX &qpos, casadi::SX &qvel) {
    typedef casadi::SX Scalar;

    // Add dynamics for fixed-cassie
    casadi::SX B(model.nv, 10);
    // TODO - Could add damping and friction effects here
    // Left Motors
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 0) = 25.0;
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 1) = 25.0;
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 2) = 16.0;
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = 16.0;
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = 50.0;
    // Right Motors
    B(model.joints[model.getJointId("RightHipYaw")].idx_v(), 5) = 25.0;
    B(model.joints[model.getJointId("RightHipRoll")].idx_v(), 6) = 25.0;
    B(model.joints[model.getJointId("RightHipPitch")].idx_v(), 7) = 16.0;
    B(model.joints[model.getJointId("RightKneePitch")].idx_v(), 8) = 16.0;
    B(model.joints[model.getJointId("RightFootPitch")].idx_v(), 9) = 50.0;

    // Return actuation matrix
    return B;
}

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