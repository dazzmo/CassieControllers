#ifndef MODEL_CLOSED_LOOP_CONSTRAINT_H
#define MODEL_CLOSED_LOOP_CONSTRAINT_H

#include <damotion/utils/eigen_wrapper.h>

#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/frames.hpp>

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
                                      const casadi::SX &qpos, const casadi::SX &qvel);

#endif /* MODEL_CLOSED_LOOP_CONSTRAINT_H */
