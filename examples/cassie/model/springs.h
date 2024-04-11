#ifndef MODEL_SPRINGS_H
#define MODEL_SPRINGS_H

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
casadi::SX CassieSpringForces(pinocchio::ModelTpl<casadi::SX> &model,
                              pinocchio::DataTpl<casadi::SX> &data,
                              const casadi::SX &qpos, const casadi::SX &qvel,
                              const double k_spring_shin = 1500.0,
                              const double k_spring_heel = 1250.0);

/**
 * @brief Computes the spring forces from the leaf-springs of Cassie, with the
 * spring constants as parameters to the constraint rather than being fixed.
 *
 * @param model Pinocchio model of a cassie robot
 * @param data Pinocchio data associated with the cassie model
 * @return casadi::SX Constraint vector for closed-loop constraint of both legs
 */
casadi::SX CassieSpringForces(pinocchio::ModelTpl<casadi::SX> &model,
                              pinocchio::DataTpl<casadi::SX> &data,
                              const casadi::SX &qpos, const casadi::SX &qvel,
                              const casadi::SX &k_spring_shin,
                              const casadi::SX &k_spring_heel);

casadi::SX CassieJointDampingForces(pinocchio::ModelTpl<casadi::SX> &model,
                                    pinocchio::DataTpl<casadi::SX> &data,
                                    const casadi::SX &qpos, const casadi::SX &qvel);
                                    
//     model.rotorInertia[cg.GetJointIdv("LeftHipRoll")] = 6.10e-05;
//     model.rotorInertia[cg.GetJointIdv("LeftHipYaw")] = 6.10e-05;
//     model.rotorInertia[cg.GetJointIdv("LeftHipPitch")] = 3.65e-04;
//     model.rotorInertia[cg.GetJointIdv("LeftKneePitch")] = 3.65e-04;
//     model.rotorInertia[cg.GetJointIdv("LeftFootPitch")] = 4.90e-06;

//     model.rotorInertia[cg.GetJointIdv("RightHipRoll")] = 6.10e-05;
//     model.rotorInertia[cg.GetJointIdv("RightHipYaw")] = 6.10e-05;
//     model.rotorInertia[cg.GetJointIdv("RightHipPitch")] = 3.65e-04;
//     model.rotorInertia[cg.GetJointIdv("RightKneePitch")] = 3.65e-04;
//     model.rotorInertia[cg.GetJointIdv("RightFootPitch")] = 4.90e-06;

#endif /* MODEL_SPRINGS_H */
