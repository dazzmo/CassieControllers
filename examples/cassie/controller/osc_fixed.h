#ifndef CONTROLLER_OSC_FIXED_H
#define CONTROLLER_OSC_FIXED_H

#include <glog/logging.h>

// pinocchio
#include <pinocchio/parsers/urdf.hpp>

// damotion
#include <damotion/control/osc/osc.h>

#include "damotion/solvers/program.h"
// model
#include "model/actuation.h"
#include "model/closed_loop_constraint.h"
#include "model/springs.h"

// Sizes
#define CASSIE_FIXED_NQ (16)
#define CASSIE_FIXED_NV (16)
#define CASSIE_FIXED_NU (10)

namespace sym = damotion::symbolic;
namespace opt = damotion::optimisation;
namespace osc = damotion::control::osc;

class CassieFixedOSC {
   public:
    CassieFixedOSC() {
        // Load cassie model
        pinocchio::Model model;
        pinocchio::urdf::buildModel("./cassie_fixed.urdf", model);
        pinocchio::Data data(model);
        // Wrap model for casadi functionality
        damotion::utils::casadi::PinocchioModelWrapper wrapper(model);

        // Create symbolic vectors for cost and constraint creation
        casadi::SX qpos = casadi::SX::sym("qpos", model.nq),
                   qvel = casadi::SX::sym("qvel", model.nv),
                   qacc = casadi::SX::sym("qacc", model.nv),
                   ctrl = casadi::SX::sym("ctrl", CASSIE_FIXED_NU);

        // Create program variables
        sym::VariableVector qacc_v =
                                sym::CreateVariableVector("qacc", model.nv),
                            ctrl_v = sym::CreateVariableVector("ctrl",
                                                               CASSIE_FIXED_NU);

        // Create mathematical program
        opt::Program program;
        program.AddDecisionVariables(qacc_v);
        program.AddDecisionVariables(ctrl_v);

        // Add parameters
        Eigen::Ref<const Eigen::MatrixXd> qpos_p = program.AddParameters(
                                              "qpos", model.nq),
                                          qvel_p = program.AddParameters(
                                              "qvel", model.nv);

        /**
         * Unconstrained Dynamics
         *
         */
        // Get actuation matrix for Cassie
        casadi::SX B =
            CassieActuationMatrix(wrapper.model(), wrapper.data(), qpos, qvel);

        // Add any additional nonlinearities (e.g. spring/damping of joints)
        casadi::SX spring_forces = CassieSpringForces(
                       wrapper.model(), wrapper.data(), qpos, qvel),
                   damping = CassieJointDampingForces(
                       wrapper.model(), wrapper.data(), qpos, qvel);

        // Compute inverse dynamics M qacc + C qvel + G
        sym::Expression dynamics =
            wrapper.rnea()(casadi::SXVector({qpos, qvel, qacc}))[0];
        // Add inputs and other features
        dynamics -= mtimes(B, ctrl);
        dynamics -= (spring_forces + damping);

        /** Register End-Effector **/

        // Add any end-effectors of interest to the model
        wrapper.addEndEffector("LeftFootFront");
        wrapper.addEndEffector("RightFootFront");

        // Compute end-effector data for end-effectors
        casadi::SXVector left_foot_x =
                             wrapper.end_effector("LeftFootFront")
                                 .x(casadi::SXVector({qpos, qvel, qacc})),
                         right_foot_x =
                             wrapper.end_effector("RightFootFront")
                                 .x(casadi::SXVector({qpos, qvel, qacc}));

        // Create end-effector data for tracking/contact
        osc::EndEffector left_foot, right_foot;
        left_foot.SetPosition(left_foot_x[0]);
        left_foot.SetVelocity(left_foot_x[1]);
        left_foot.SetAcceleration(left_foot_x[2]);

        right_foot.SetPosition(right_foot_x[0]);
        right_foot.SetVelocity(right_foot_x[1]);
        right_foot.SetAcceleration(right_foot_x[2]);

        left_foot.GenerateFunction("left_foot", true, "./");
        right_foot.GenerateFunction("right_foot", true, "./");

        /**
         * Tracking Tasks
         *
         */

        // Create tracking tasks for any desired end-effectors
        tracking_tasks_["foot_l"] = osc::TrackingTaskData();
        tracking_tasks_["foot_r"] = osc::TrackingTaskData();

        // Add parameters to program

        std::vector<osc::EndEffector> ee = {left_foot, right_foot};

        // Add tracking costs
        for (osc::EndEffector e : ee) {
            // Create parameter for program for acceleration tracking
            Eigen::Ref<const Eigen::MatrixXd> xacc_d_p =
                program.AddParameters(e.name() + "_xaccd", e.Dimension());
            // Create objective for tracking
            casadi::SX xacc_d = casadi::SX::sym("xacc_d", e.Dimension());
            sym::Expression obj =
                osc::TaskAccelerationErrorCost(e.Acceleration(), xacc_d);
            obj.SetInputs({qacc}, {qpos, qvel, xacc_d});
            // Add objective to program
            program.AddCost(obj, {qacc_v}, {qpos_p, qvel_p, xacc_d_p});
        }

        /**
         * Contact Tasks
         *
         */

        /**
         * Holonomic Constraints
         *
         */

        casadi::SX cl = CassieClosedLoopConstraint(wrapper.model(),
                                                   wrapper.data(), qpos, qvel);

        // Compute first and second derivatives
        casadi::SX Jcl = jacobian(cl, qpos);
        casadi::SX dJcldt = jacobian(mtimes(Jcl, qvel), qpos);

        casadi::SX dcl = mtimes(Jcl, qvel);
        casadi::SX ddcl = mtimes(Jcl, qacc) + mtimes(dJcldt, qvel);

        // Convert to linear constraint in qacc
        casadi::SX A, b;
        osc::NoSlipConstraintCoefficients(ddcl, qacc, A, b);
        auto no_slip = std::make_shared<opt::LinearConstraint>(
            A, b, casadi::SXVector({qpos, qvel}));
        // Add linear constraint to program, binding variables and constraints
        // to it
        program.AddLinearConstraint(no_slip, {qacc_v}, {qpos_p, qvel_p});

        /**
         *
         * Other Costs
         *
         */

        // Add a torque-regularisation cost
        sym::Expression u2 = casadi::SX::dot(ctrl, ctrl);
        u2.SetInputs({ctrl}, {});
        program.AddCost(u2, {ctrl_v}, {});

        std::cout << "Added Everything\n";

        /**
         * Decision Variable Vector
         *
         */

        // Create optimisation vector
        sym::VariableVector x(qacc_v.size() + ctrl_v.size());
        x << qacc_v, ctrl_v;
        program.SetDecisionVariableVector(x);

        /**
         * Bounds
         *
         */

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

        // Add bounds to the control variables
        // osc_.GetDecisionVariables("ctrl").UpperBound()
        // << 4.5, 4.5, 12.2, 12.2,
        //     0.9, 4.5, 4.5, 12.2, 12.2, 0.9;
        // osc_.GetDecisionVariables("ctrl").LowerBound() << -4.5, -4.5, -12.2,
        //     -12.2, -0.9, -4.5, -4.5, -12.2, -12.2, -0.9;

        // osc_.GetDecisionVariables("qacc").UpperBound().setConstant(1e4);
        // osc_.GetDecisionVariables("qacc").LowerBound().setConstant(-1e4);

        // osc_.UpdateDecisionVariableVectorBounds("ctrl");
        // osc_.UpdateDecisionVariableVectorBounds("qacc");

        // Set up any tracking parameters
        tracking_tasks_["foot_l"].Kp.diagonal().setOnes();
        tracking_tasks_["foot_l"].Kd.diagonal().setOnes();

        tracking_tasks_["foot_r"].Kp.diagonal().setOnes();
        tracking_tasks_["foot_r"].Kd.diagonal().setOnes();

        // Show the program details
        osc_.PrintProgramSummary();
        // Print detailed list of variables and constraints
        osc_.ListVariables();
        osc_.ListParameters();
        osc_.ListCosts();
        osc_.ListConstraints();
    }

    ~CassieFixedOSC() = default;

    // Update the references for any tasks
    void UpdateReferences(double time, const Eigen::VectorXd& qpos,
                          const Eigen::VectorXd& qvel) {
        double l_phase = -(2.0 * M_PI / 4.0) * time;

        Eigen::Vector3d &xl = tracking_task_["LeftFootFront"].xr,
                        &xr = tracking_task_["RightFootFront"].xr;

        xl[0] = 0.0 + 0.2 * cos(l_phase);
        xl[1] = 0.1;
        xl[2] = -0.7 + 0.2 * sin(l_phase);

        xr[0] = 0.0 + 0.2 * cos(l_phase);
        xr[1] = 0.1;
        xr[2] = -0.7 + 0.2 * sin(l_phase);

        // Compute new tracking errors
        // osc::GetTaskError();

        program.SetParameter("left_foot_xaccd", xl);
        program.SetParameter("right_foot_xaccd", xr);
    }

    // Update controller state
    void UpdateState(int nq, const double* q, int nv, const double* v);

    void Solve() {
        // Update program parameters
        osc_.UpdateProgramParameters();
        // Update solver with current program
        solver_->UpdateProgram(osc_);
        // Call a solver for the created program
        // solver_->Solve();
    }

   protected:
   private:
    // Solver
    std::unique_ptr<damotion::optimisation::solvers::SolverBase> solver_;

    std::unordered_map<std::string, osc::TrackingTaskData> tracking_tasks_;
    // std::unordered_map<std::string, osc::ContactTask> contact_tasks_;
};

#endif /* CONTROLLER_OSC_FIXED_H */
