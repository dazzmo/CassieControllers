#ifndef CONTROLLER_OSC_FIXED_H
#define CONTROLLER_OSC_FIXED_H

#include <glog/logging.h>

// pinocchio
#include <pinocchio/parsers/urdf.hpp>

// damotion
#include <damotion/control/osc/osc.h>
#include <damotion/solvers/solve_qpoases.h>

#include "damotion/solvers/bounds.h"
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

        // Create symbolic variables for constraint/cost creation
        std::unordered_map<std::string, casadi::SX> sym_var;
        sym_var["qpos"] = casadi::SX::sym("qpos", model.nq);
        sym_var["qvel"] = casadi::SX::sym("qvel", model.nv);
        sym_var["qacc"] = casadi::SX::sym("qacc", model.nv);
        sym_var["ctrl"] = casadi::SX::sym("ctrl", CASSIE_FIXED_NU);

        qpos_ = Eigen::VectorXd(model.nq);
        qvel_ = Eigen::VectorXd(model.nq);

        // Create program variables
        qacc_ = sym::CreateVariableVector("qacc", model.nv),
        ctrl_ = sym::CreateVariableVector("ctrl", CASSIE_FIXED_NU);

        // Create mathematical program
        program_ = opt::Program("osc");
        program_.AddDecisionVariables(qacc_);
        program_.AddDecisionVariables(ctrl_);

        // Add parameters
        Eigen::Ref<const Eigen::MatrixXd> qpos = program_.AddParameters(
                                              "qpos", model.nq),
                                          qvel = program_.AddParameters(
                                              "qvel", model.nv);

        /**
         * Unconstrained Dynamics
         *
         */
        // Get actuation matrix for Cassie
        casadi::SX B = CassieActuationMatrix(wrapper.model(), wrapper.data(),
                                             sym_var["qpos"], sym_var["qvel"]);

        // Add any additional nonlinearities (e.g. spring/damping of joints)
        casadi::SX spring_forces =
                       CassieSpringForces(wrapper.model(), wrapper.data(),
                                          sym_var["qpos"], sym_var["qvel"]),
                   damping = CassieJointDampingForces(
                       wrapper.model(), wrapper.data(), sym_var["qpos"],
                       sym_var["qvel"]);

        // Compute inverse dynamics M qacc + C qvel + G
        sym::Expression dynamics = wrapper.rnea()(casadi::SXVector(
            {sym_var["qpos"], sym_var["qvel"], sym_var["qacc"]}))[0];
        // Add inputs and other features
        dynamics -= mtimes(B, sym_var["ctrl"]);
        dynamics -= (spring_forces + damping);

        /** Register End-Effectors **/

        // Add any end-effectors of interest to the model
        wrapper.addEndEffector("LeftFootFront");
        wrapper.addEndEffector("RightFootFront");

        // Compute end-effector data for end-effectors
        casadi::SXVector foot_l_x = wrapper.end_effector("LeftFootFront")
                                        .x(casadi::SXVector({sym_var["qpos"],
                                                             sym_var["qvel"],
                                                             sym_var["qacc"]})),
                         foot_r_x = wrapper.end_effector("RightFootFront")
                                        .x(casadi::SXVector({sym_var["qpos"],
                                                             sym_var["qvel"],
                                                             sym_var["qacc"]}));

        // Create end-effector data for tracking/contact
        osc::EndEffector foot_l("foot_l"), foot_r("foot_r");
        // Create expressions for the end-effectors
        foot_l.SetPosition(foot_l_x[0]);
        foot_l.SetVelocity(foot_l_x[1]);
        foot_l.SetAcceleration(foot_l_x[2]);

        foot_r.SetPosition(foot_r_x[0]);
        foot_r.SetVelocity(foot_r_x[1]);
        foot_r.SetAcceleration(foot_r_x[2]);

        // Set inputs for the expressions for the state of the end-effectors
        foot_l.SetInputs({sym_var["qacc"]}, {sym_var["qpos"], sym_var["qvel"]});
        foot_r.SetInputs({sym_var["qacc"]}, {sym_var["qpos"], sym_var["qvel"]});

        foot_l.GenerateFunction("foot_l", true, "./");
        foot_r.GenerateFunction("foot_r", true, "./");

        // Add these to the map
        ee_idx_[foot_l.id()] = ee_.size();
        ee_.push_back(foot_l);
        ee_idx_[foot_r.id()] = ee_.size();
        ee_.push_back(foot_r);

        /**
         * Tracking Tasks
         *
         */

        // Create tracking data for each foot
        osc::TrackingTaskData foot_l_track, foot_r_track;

        foot_l_track.end_effector_id = foot_l.id();
        foot_l_track.type = osc::TrackingTaskData::Type::kTranslational;

        foot_r_track.end_effector_id = foot_r.id();
        foot_r_track.type = osc::TrackingTaskData::Type::kTranslational;

        tracking_tasks_["foot_l"] = foot_l_track;
        tracking_tasks_["foot_r"] = foot_r_track;

        // Add tracking costs
        for (auto& task : tracking_tasks_) {
            // Get task
            std::string name = task.first;
            osc::TrackingTaskData& data = task.second;
            osc::EndEffector& ee = ee_[ee_idx_[data.end_effector_id]];

            data.Kp.resize(3);
            data.Kd.resize(3);

            data.Kp.diagonal().setOnes();
            data.Kd.diagonal().setOnes();

            // Create parameter for program for acceleration tracking
            Eigen::Ref<const Eigen::MatrixXd> xaccd =
                program_.AddParameters(name + "_xaccd", 3);

            // Create objective for tracking
            casadi::SX sym_xaccd = casadi::SX::sym("xacc_d", 3);
            sym::Expression obj = osc::TaskAccelerationErrorCost(
                ee.Acceleration()(casadi::Slice(0, 3)), sym_xaccd);
            obj *= 1e2;
            obj.SetInputs({sym_var["qacc"]},
                          {sym_var["qpos"], sym_var["qvel"], sym_xaccd});

            // Add objective to program
            std::shared_ptr<opt::QuadraticCost> task_cost =
                std::make_shared<opt::QuadraticCost>(obj, name + "_tracking");
            program_.AddQuadraticCost(task_cost, {qacc_}, {qpos, qvel, xaccd});
        }

        std::cout << "Tracking\n";

        /**
         * Contact Tasks
         *
         */

        /**
         * Holonomic Constraints
         *
         */

        casadi::SX cl = CassieClosedLoopConstraint(
            wrapper.model(), wrapper.data(), sym_var["qpos"], sym_var["qvel"]);
        // Create holonomic constraint
        osc::HolonomicConstraint closed_loop("cassie_closed_loop");

        // Compute first and second derivatives
        casadi::SX Jcl = jacobian(cl, sym_var["qpos"]);
        casadi::SX dJcldt =
            jacobian(mtimes(Jcl, sym_var["qvel"]), sym_var["qpos"]);

        closed_loop.SetConstraint(cl);
        closed_loop.SetConstraintFirstDerivative(mtimes(Jcl, sym_var["qvel"]));
        closed_loop.SetConstraintSecondDerivative(
            mtimes(Jcl, sym_var["qacc"]) + mtimes(dJcldt, sym_var["qvel"]));

        // Add constraint forces
        sym::VariableVector lam_closed_loop =
            sym::CreateVariableVector("lam_closed_loop", cl.size1());
        program_.AddDecisionVariables(lam_closed_loop);
        // Add bounds to constraint forces
        program_.AddBoundingBoxConstraint(-1e3, 1e3, lam_closed_loop);

        constraints_.push_back(closed_loop);
        constraint_forces_.push_back(lam_closed_loop);

        // Convert to linear constraint in qacc
        casadi::SX A, b;
        osc::NoSlipConstraintCoefficients(
            closed_loop.ConstraintSecondDerivative(), sym_var["qacc"], A, b);
        auto no_slip = std::make_shared<opt::LinearConstraint>(
            A, b, casadi::SXVector({sym_var["qpos"], sym_var["qvel"]}),
            opt::BoundsType::kEquality, "closed_loop");

        // Add linear constraint to program, binding variables and constraints
        // to it
        program_.AddLinearConstraint(no_slip, {qacc_}, {qpos, qvel});

        std::cout << "Constraints\n";

        /**
         *
         *  Constrained dynamics
         *
         */
        casadi::SXVector in = {sym_var["qacc"], sym_var["ctrl"]};
        sym::VariableVector in_dyn(qacc_.size() + ctrl_.size());
        in_dyn << qacc_, ctrl_;

        // For each constraint
        for (int i = 0; i < constraints_.size(); i++) {
            in_dyn.conservativeResize(in_dyn.size() +
                                      constraint_forces_[i].size());
            in_dyn.bottomRows(constraint_forces_[i].size())
                << constraint_forces_[i];
            casadi::SX lam =
                casadi::SX::sym("lam", constraint_forces_[i].size());
            in.push_back(lam);
            casadi::SX J = jacobian(constraints_[i].ConstraintFirstDerivative(),
                                    sym_var["qvel"]);
            dynamics -= mtimes(J.T(), lam);
        }
        // For each contact point
        for (int i = 0; i < contact_tasks_.size(); i++) {
            in_dyn.conservativeResize(in_dyn.size() +
                                      contact_forces_[i].size());
            in_dyn.bottomRows(contact_forces_[i].size()) << contact_forces_[i];
            casadi::SX lam = casadi::SX::sym("lam", contact_forces_[i].size());
            in.push_back(lam);
            casadi::SX J = jacobian(ee_[i].Velocity(), sym_var["qvel"]);
            dynamics -= mtimes(J.T(), lam);
        }

        std::cout << "Dynamics\n";

        // Create single vector for linear expression
        // Set inputs for expression
        dynamics.SetInputs({vertcat(in)}, {sym_var["qpos"], sym_var["qvel"]});
        // Add constraint with variable bindings
        casadi::SX::linear_coeff(dynamics, vertcat(in), A, b, true);
        std::shared_ptr<opt::LinearConstraint> dynamics_con =
            std::make_shared<opt::LinearConstraint>(A, b, dynamics.Parameters(),
                                                    opt::BoundsType::kEquality,
                                                    "dynamics");
        program_.AddLinearConstraint(dynamics_con, {in_dyn}, {qpos, qvel});

        /**
         *
         * Other Costs
         *
         */

        // Joint damping
        casadi::SX damping_task_error = sym_var["qacc"] - sym_var["qvel"];
        sym::Expression joint_damping =
            casadi::SX::dot(damping_task_error, damping_task_error);

        // Add a torque-regularisation cost
        sym::Expression u2 =
            1e-4 * casadi::SX::dot(sym_var["ctrl"], sym_var["ctrl"]);
        u2.SetInputs({sym_var["ctrl"]}, {});
        std::shared_ptr<opt::QuadraticCost> pu2 =
            std::make_shared<opt::QuadraticCost>(u2, "torque-cost");
        program_.AddQuadraticCost(pu2, {ctrl_}, {});

        std::cout << "Costs\n";

        /**
         * Decision Variable Vector
         *
         */

        // Create optimisation vector [qacc, ctrl, lambda]
        sym::VariableVector x(program_.NumberOfDecisionVariables());
        // Set it as qacc, ctrl, constraint forces
        x.topRows(qacc_.size() + ctrl_.size()) << qacc_, ctrl_;
        int idx = qacc_.size() + ctrl_.size();
        for (int i = 0; i < constraint_forces_.size(); ++i) {
            x.middleRows(idx, constraint_forces_[i].size()) =
                constraint_forces_[i];
        }
        program_.SetDecisionVariableVector(x);
        std::cout << "Vector\n";

        /**
         * Bounds
         *
         */
        // Joint accelerations
        Eigen::VectorXd lb_qacc(model.nv), ub_qacc(model.nv);
        lb_qacc.setConstant(-1e4);
        ub_qacc.setConstant(1e4);
        program_.AddBoundingBoxConstraint(lb_qacc, ub_qacc, qacc_);
        // Control inputs
        Eigen::VectorXd lb_ctrl(CASSIE_FIXED_NU), ub_ctrl(CASSIE_FIXED_NU);
        ub_ctrl << 4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2, 0.9;
        lb_ctrl = -ub_ctrl;
        program_.AddBoundingBoxConstraint(lb_ctrl, ub_ctrl, ctrl_);

        std::cout << "Bounds\n";

        // Show the program details
        program_.PrintProgramSummary();
        // Print detailed list of variables and constraints
        program_.ListDecisionVariables();
        program_.ListParameters();
        program_.ListCosts();
        program_.ListConstraints();

        // Create solver for the program
        solver_ =
            std::make_unique<opt::solvers::QPOASESSolverInstance>(program_);
    }

    ~CassieFixedOSC() = default;

    // Update the references for any tasks
    void UpdateReferences(double time);

    // Update controller state
    void UpdateState(int nq, const double* q, int nv, const double* v);

    void Solve() {
        solver_->UpdateProgram(program_);
        solver_->Solve();
    }

    Eigen::VectorXd CurrentControlSolution() {
        return solver_->GetVariableValues(ctrl_);
    }

   protected:
   private:
    // Program
    opt::Program program_;
    // Solver
    std::unique_ptr<opt::solvers::QPOASESSolverInstance> solver_;

    Eigen::VectorXd qpos_;
    Eigen::VectorXd qvel_;

    std::vector<osc::EndEffector> ee_;
    std::unordered_map<osc::EndEffector::Id, int> ee_idx_;

    // Standard optimisation variables
    sym::VariableVector qacc_;
    sym::VariableVector ctrl_;
    // Vector of contact-force variables
    std::vector<sym::VariableVector> contact_forces_;
    // Vector of other constraint-based variables
    std::vector<sym::VariableVector> constraint_forces_;

    // Tasks
    std::unordered_map<std::string, osc::TrackingTaskData> tracking_tasks_;
    std::unordered_map<std::string, osc::ContactTaskData> contact_tasks_;
    std::vector<osc::HolonomicConstraint> constraints_;
};

#endif /* CONTROLLER_OSC_FIXED_H */
