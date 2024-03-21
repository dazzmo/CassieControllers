#ifndef OSC_FIXED_HPP
#define OSC_FIXED_HPP

#include <glog/logging.h>

// pinocchio
#include <pinocchio/parsers/urdf.hpp>

// damotion
#include <damotion/control/osc/osc.h>

// model
#include "model/closed_loop_constraint.h"

// Sizes
#define CASSIE_FIXED_NQ (16)
#define CASSIE_FIXED_NV (16)
#define CASSIE_FIXED_NU (10)

class CassieFixedOSC {
   public:
    CassieFixedOSC() {
        // Load cassie model
        pinocchio::Model model;
        pinocchio::urdf::buildModel("./cassie_fixed.urdf", model, true);
        pinocchio::Data data(model);
        // Wrap model
        casadi_utils::PinocchioModelWrapper wrapper(model);

        // Create OSC
        osc_ = damotion::control::OSCController(model.nq, model.nv,
                                                CASSIE_FIXED_NU);

        // Get variables from the program
        casadi::SX qpos = osc_.GetParameters("qpos"),
                   qvel = osc_.GetParameters("qvel"),
                   qacc = osc_.GetVariables("qacc"),
                   ctrl = osc_.GetVariables("ctrl");

        // Add dynamics for fixed-cassie
        casadi::SX B(CASSIE_FIXED_NV, CASSIE_FIXED_NU);

        // Compute inverse dynamics M qacc + C qvel + G
        casadi::SX dyn =
            wrapper.rnea()(casadi::SXVector({qpos, qvel, qacc}))[0];

        // Add any additional nonlinearities (e.g. spring/damping of joints)
        casadi::SX spring_forces = casadi::SX::zeros(CASSIE_FIXED_NV),
                   damping = casadi::SX::zeros(CASSIE_FIXED_NV);
        // TODO - We could add these variables as parameters to the program with
        // TODO - osc_.AddParameter() which can be adjusted during optimisation
        double k_heel = 2400.0, k_achilles = 2000.0;  // Spring Constants (N/m)

        spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) =
            k_heel *
            qpos(model.joints[model.getJointId("LeftShinPitch")].idx_q());
        spring_forces(
            model.joints[model.getJointId("RightShinPitch")].idx_v()) =
            k_heel *
            qpos(model.joints[model.getJointId("RightShinPitch")].idx_q());
        spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) =
            k_heel *
            qpos(model.joints[model.getJointId("LeftShinPitch")].idx_q());
        spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) =
            k_heel *
            qpos(model.joints[model.getJointId("LeftShinPitch")].idx_q());

        // Add generalised inputs
        dyn -= mtimes(B, ctrl) - spring_forces - damping;

        // Create new function with actuation included
        casadi::Function dynamics =
            casadi::Function("dynamics", {qpos, qvel, qacc, ctrl}, {dyn},
                             {"qpos", "qvel", "qacc", "ctrl"}, {"dyn"});

        osc_.AddDynamics(dynamics);

        // Add any end-effectors
        wrapper.addEndEffector("LeftFootFront");
        wrapper.addEndEffector("RightFootFront");

        osc_.AddTrackingTask(
            "LeftFootFront",
            wrapper.end_effector(wrapper.end_effector_idx("LeftFootFront")).x,
            damotion::control::OSCController::TrackingTask::Type::kFull);

        osc_.AddTrackingTask(
            "RightFootFront",
            wrapper.end_effector(wrapper.end_effector_idx("RightFootFront")).x,
            damotion::control::OSCController::TrackingTask::Type::kFull);

        /* Holonomic constraints */
        casadi::SX cl = CassieClosedLoopConstraint(wrapper.model(),
                                                   wrapper.data(), qpos, qvel);

        // Compute first and second derivatives
        casadi::SX Jcl = jacobian(cl, qpos);
        casadi::SX dJcldt = jacobian(mtimes(Jcl, qvel), qpos);

        casadi::SX dcl = mtimes(Jcl, qvel);
        casadi::SX ddcl = mtimes(Jcl, qacc) + mtimes(dJcldt, qvel);
        // Create function
        casadi::Function fcl("closed_loop", {qpos, qvel, qacc}, {cl, dcl, ddcl},
                             {"qpos", "qvel", "qacc"}, {"cl", "dcl", "ddcl"});

        // Add to OSC
        osc_.AddHolonomicConstraint("closed_loop", fcl);

        // Initialise program with given constraints
        osc_.Initialise();

        // // Order: hip roll, yaw, pitch, knee, shin (spring), tarsus, heel
        // // spring, toe
        // initial_state().q << 0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0,
        //     -1.5968, -0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0,
        //     -1.5968;

        // // Add bounds (from cassie.xml, different from kinematic model on
        // wiki) bounds().qmin << -15.0, -22.5, -50.0, -164.0, -20.0, 50.0,
        // -20.0,
        //     -140.0, -15.0, -22.5, -50.0, -164.0, -20.0, 50.0, -20.0, -140.0;
        // bounds().qmax << 22.5, 22.5, 80.0, -37.0, 20.0, 170.0, 20.0, -30.0,
        //     22.5, 22.5, 80.0, -37.0, 20.0, 170.0, 20.0, -30.0;
        // bounds().qmin *= M_PI / 180;
        // bounds().qmax *= M_PI / 180;

        // Add bounds to the control variables
        // TODO - Update variable bounds

        // bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2,
        // 0.9; bounds().vmax << 12.15, 12.15, 8.5, 8.5, 20, 20,
        // 20, 11.52, 12.15, 12.15, 8.5, 8.5, 20, 20, 20, 11.52;  // From
        // cassie.urdf bounds().amax.setConstant(1e4);          // This is a
        // guess

        // Show the program details
        osc_.PrintProgramSummary();
    }

    ~CassieFixedOSC() = default;

    // TODO - Make a base class that controllers can inherit the cassie OSC from
    // TODO - and come up with their own custom reference tracking and stuff, so
    // TODO - multiple controllers can be made

    // Update the references for any tasks
    void UpdateReferences(double time, const Eigen::VectorXd& qpos,
                          const Eigen::VectorXd& qvel) {
        double l_phase = -(2.0 * M_PI / 4.0) * time;
        // Get reference vectors for foot positions
        Eigen::Vector3d &xl = osc_.GetTrackingTask("LeftFootFront").xr,
                        &xr = osc_.GetTrackingTask("RightFootFront").xr;

        xl[0] = 0.0 + 0.2 * cos(l_phase);
        xl[1] = 0.1;
        xl[2] = -0.7 + 0.2 * sin(l_phase);

        xr[0] = 0.0 + 0.2 * cos(l_phase);
        xr[1] = 0.1;
        xr[2] = -0.7 + 0.2 * sin(l_phase);
    }

    // Update controller state
    void UpdateState(int nq, const double* q, int nv, const double* v);

   protected:
   private:
    damotion::control::OSCController osc_;
    // Solver
};

#endif /* OSC_FIXED_HPP */
