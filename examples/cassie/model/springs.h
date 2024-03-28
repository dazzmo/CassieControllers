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
                              casadi::SX &qpos, casadi::SX &qvel,
                              const double k_spring_shin = 1500.0,
                              const double k_spring_heel = 1250.0) {
    typedef casadi::SX Scalar;

    // Add any additional nonlinearities (e.g. spring/damping of joints)
    casadi::SX spring_forces = casadi::SX::zeros(model.nv);

    // TODO - Work out the typing of these joints
    auto shin_l = model.joints[model.getJointId("LeftShinPitch")],
         shin_r = model.joints[model.getJointId("RightShinPitch")],
         heel_l = model.joints[model.getJointId("LeftAchillesSpring")],
         heel_r = model.joints[model.getJointId("RightAchillesSpring")];

    spring_forces(shin_l.idx_v()) = k_spring_shin * qpos(shin_l.idx_q());
    spring_forces(shin_r.idx_v()) = k_spring_shin * qpos(shin_r.idx_q());
    spring_forces(heel_l.idx_v()) = k_spring_heel * qpos(heel_l.idx_q());
    spring_forces(heel_r.idx_v()) = k_spring_heel * qpos(heel_r.idx_q());

    // Return actuation matrix
    return spring_forces;
}

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
                              casadi::SX &qpos, casadi::SX &qvel,
                              const casadi::SX &k_spring_shin,
                              const casadi::SX &k_spring_heel) {
    typedef casadi::SX Scalar;

    // Add any additional nonlinearities (e.g. spring/damping of joints)
    casadi::SX spring_forces = casadi::SX::zeros(model.nv);

    // TODO - Work out the typing of these joints
    auto shin_l = model.joints[model.getJointId("LeftShinPitch")],
         shin_r = model.joints[model.getJointId("RightShinPitch")],
         heel_l = model.joints[model.getJointId("LeftAchillesSpring")],
         heel_r = model.joints[model.getJointId("RightAchillesSpring")];

    spring_forces(shin_l.idx_v()) = k_spring_shin * qpos(shin_l.idx_q());
    spring_forces(shin_r.idx_v()) = k_spring_shin * qpos(shin_r.idx_q());
    spring_forces(heel_l.idx_v()) = k_spring_heel * qpos(heel_l.idx_q());
    spring_forces(heel_r.idx_v()) = k_spring_heel * qpos(heel_r.idx_q());

    // Return actuation matrix
    return spring_forces;
}

casadi::SX CassieJointDampingForces(pinocchio::ModelTpl<casadi::SX> &model,
                                    pinocchio::DataTpl<casadi::SX> &data,
                                    casadi::SX &qpos, casadi::SX &qvel) {
    typedef casadi::SX Scalar;

    // Add any additional nonlinearities (e.g. spring/damping of joints)
    casadi::SX damping = casadi::SX::zeros(model.nv);

    // TODO - Work out the typing of these joints
    // Left leg
    auto hip_l_r = model.joints[model.getJointId("LeftHipRoll")],
         hip_l_y = model.joints[model.getJointId("LeftHipYaw")],
         hip_l_p = model.joints[model.getJointId("LeftHipPitch")],
         knee_l = model.joints[model.getJointId("LeftKneePitch")],
         shin_l = model.joints[model.getJointId("LeftShinPitch")],
         tarsus_l = model.joints[model.getJointId("LeftTarsusPitch")],
         achilles_l = model.joints[model.getJointId("LeftAchillesSpring")],
         foot_l = model.joints[model.getJointId("LeftFootPitch")],
         // Right leg
        hip_r_r = model.joints[model.getJointId("RightHipRoll")],
         hip_r_y = model.joints[model.getJointId("RightHipYaw")],
         hip_r_p = model.joints[model.getJointId("RightHipPitch")],
         knee_r = model.joints[model.getJointId("RightKneePitch")],
         shin_r = model.joints[model.getJointId("RightShinPitch")],
         tarsus_r = model.joints[model.getJointId("RightTarsusPitch")],
         achilles_r = model.joints[model.getJointId("RightAchillesSpring")],
         foot_r = model.joints[model.getJointId("RightFootPitch")];

    // Left leg
    damping(hip_l_r.idx_v()) = 1.0 * qvel(hip_l_r.idx_v());
    damping(hip_l_y.idx_v()) = 1.0 * qvel(hip_l_y.idx_v());
    damping(hip_l_p.idx_v()) = 1.0 * qvel(hip_l_p.idx_v());
    damping(knee_l.idx_v()) = 1.0 * qvel(knee_l.idx_v());
    damping(shin_l.idx_v()) = 0.1 * qvel(shin_l.idx_v());
    damping(tarsus_l.idx_v()) = 0.1 * qvel(tarsus_l.idx_v());
    damping(achilles_l.idx_v()) = 0.001 * qvel(achilles_l.idx_v());
    damping(foot_l.idx_v()) = 1.0 * qvel(foot_l.idx_v());

    // Right leg
    damping(hip_r_r.idx_v()) = 1.0 * qvel(hip_r_r.idx_v());
    damping(hip_r_y.idx_v()) = 1.0 * qvel(hip_r_y.idx_v());
    damping(hip_r_p.idx_v()) = 1.0 * qvel(hip_r_p.idx_v());
    damping(knee_r.idx_v()) = 1.0 * qvel(knee_r.idx_v());
    damping(shin_r.idx_v()) = 0.1 * qvel(shin_r.idx_v());
    damping(tarsus_r.idx_v()) = 0.1 * qvel(tarsus_r.idx_v());
    damping(achilles_r.idx_v()) = 0.001 * qvel(achilles_r.idx_v());
    damping(foot_r.idx_v()) = 1.0 * qvel(foot_r.idx_v());

    // Return actuation matrix
    return damping;
}

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