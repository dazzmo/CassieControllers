#ifndef MODEL_CLOSED_LOOP_CONSTRAINT_H
#define MODEL_CLOSED_LOOP_CONSTRAINT_H

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
casadi::SX CassieClosedLoopConstraint(pinocchio::ModelTpl<casadi::SX> &model,
                                      pinocchio::DataTpl<casadi::SX> &data,
                                      const casadi::SX &qpos, const casadi::SX &qvel) {
    typedef casadi::SX Scalar;

    // Create reference frames for the closed-loop constraint locations (two
    // points on each leg to be constrained a certain distance apart)

    // https://github.com/agilityrobotics/cassie-doc/wiki/Heel-Spring-Model

    // Left spring tip
    model.addFrame(pinocchio::FrameTpl<Scalar>(
        "left_heel_spring_tip", model.getJointId("LeftAchillesSpring"), 0,
        pinocchio::SE3Tpl<Scalar>(Eigen::Matrix3<Scalar>::Identity(),
                                  Eigen::Vector3<Scalar>(0.11877, -0.01, 0.0)),
        pinocchio::OP_FRAME));
    // Right spring tip
    model.addFrame(pinocchio::FrameTpl<Scalar>(
        "right_heel_spring_tip", model.getJointId("RightAchillesSpring"), 0,
        pinocchio::SE3Tpl<Scalar>(Eigen::Matrix3<Scalar>::Identity(),
                                  Eigen::Vector3<Scalar>(0.11877, -0.01, 0.0)),
        pinocchio::OP_FRAME));

    // https://github.com/agilityrobotics/cassie-doc/wiki/Thigh-Model

    // Socket of the left achilles rod
    model.addFrame(pinocchio::FrameTpl<Scalar>(
        "left_achilles_rod_socket", model.getJointId("LeftHipPitch"), 0,
        pinocchio::SE3Tpl<Scalar>(Eigen::Matrix3<Scalar>::Identity(),
                                  Eigen::Vector3<Scalar>(0.0, 0.0, 0.045)),
        pinocchio::OP_FRAME));

    // Socket of the right achilles rod
    model.addFrame(pinocchio::FrameTpl<Scalar>(
        "right_achilles_rod_socket", model.getJointId("RightHipPitch"), 0,
        pinocchio::SE3Tpl<Scalar>(Eigen::Matrix3<Scalar>::Identity(),
                                  Eigen::Vector3<Scalar>(0.0, 0.0, 0.045)),
        pinocchio::OP_FRAME));

    // Update data for new model frames
    data = pinocchio::DataTpl<Scalar>(model);

    // Get model and data references, update forward kinematics
    // Convert
    Eigen::VectorX<Scalar> qpos_e;
    damotion::utils::casadi::toEigen(qpos, qpos_e);
    pinocchio::framesForwardKinematics(model, data, qpos_e);

    // Get distance of the two parts
    Eigen::Vector3<Scalar> dl =
        data.oMf[model.getFrameId("left_achilles_rod_socket")].translation() -
        data.oMf[model.getFrameId("left_heel_spring_tip")].translation();

    Eigen::Vector3<Scalar> dr =
        data.oMf[model.getFrameId("right_achilles_rod_socket")].translation() -
        data.oMf[model.getFrameId("right_heel_spring_tip")].translation();

    // Length of the achilles rods (m)
    double achilles_rod_length = 0.5012;

    auto shin_l = model.joints[model.getJointId("LeftShinPitch")],
         shin_r = model.joints[model.getJointId("RightShinPitch")],
         achilles_l = model.joints[model.getJointId("LeftAchillesSpring")],
         achilles_r = model.joints[model.getJointId("RightAchillesSpring")];

    casadi::SX cl = casadi::SX::vertcat(
        {dl.squaredNorm() - achilles_rod_length * achilles_rod_length,
         dr.squaredNorm() - achilles_rod_length * achilles_rod_length,
         // Also zero the spring deflections
         qpos(shin_l.idx_q()), qpos(shin_r.idx_q()), qpos(achilles_l.idx_q()),
         qpos(achilles_r.idx_q())});

    // Return constraint
    return cl;
}

#endif /* MODEL_CLOSED_LOOP_CONSTRAINT_H */
