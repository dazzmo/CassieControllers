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
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 0) = 25.0;
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 1) = 25.0;
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 2) = 16.0;
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = 16.0;
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = 50.0;
    // Right Motors
    B(model.joints[model.getJointId("RightHipPitch")].idx_v(), 5) = 25.0;
    B(model.joints[model.getJointId("RightHipRoll")].idx_v(), 6) = 25.0;
    B(model.joints[model.getJointId("RightHipYaw")].idx_v(), 7) = 16.0;
    B(model.joints[model.getJointId("RightKneePitch")].idx_v(), 8) = 16.0;
    B(model.joints[model.getJointId("RightFootPitch")].idx_v(), 9) = 50.0;

    // Return actuation matrix
    return B;
}

#endif /* MODEL_ACTUATION */
