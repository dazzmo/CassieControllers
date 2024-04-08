#include "osc_fixed.h"

CassieFixedOSC::CassieFixedOSC() {
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
    Eigen::Ref<const Eigen::MatrixXd> qpos = program_.AddParameters("qpos",
                                                                    model.nq),
                                      qvel = program_.AddParameters("qvel",
                                                                    model.nv);

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
               damping =
                   CassieJointDampingForces(wrapper.model(), wrapper.data(),
                                            sym_var["qpos"], sym_var["qvel"]);

    // Compute inverse dynamics M qacc + C qvel + G
    sym::Expression dynamics = wrapper.rnea()(casadi::SXVector(
        {sym_var["qpos"], sym_var["qvel"], sym_var["qacc"]}))[0];
    // Add inputs and other features
    dynamics -= mtimes(B, sym_var["ctrl"]);
    dynamics += (spring_forces + damping);

    /** Register End-Effectors **/

    // Add any end-effectors of interest to the model
    wrapper.addEndEffector("LeftFootFront");
    wrapper.addEndEffector("RightFootFront");

    // Compute end-effector data for end-effectors
    casadi::SXVector ee_input =
        casadi::SXVector({sym_var["qpos"], sym_var["qvel"], sym_var["qacc"]});
    casadi::SXVector foot_l_x =
                         wrapper.end_effector("LeftFootFront").x(ee_input),
                     foot_r_x =
                         wrapper.end_effector("RightFootFront").x(ee_input);

    // Create end-effectors
    osc::EndEffectorFactory ee_factory;
    std::shared_ptr<osc::EndEffector> foot_l = ee_factory.Create("foot_l"),
                                      foot_r = ee_factory.Create("foot_r");

    // Add expressions for the end-effectors
    foot_l->SetPosition(foot_l_x[0]);
    foot_l->SetVelocity(foot_l_x[1]);
    foot_l->SetAcceleration(foot_l_x[2]);

    foot_r->SetPosition(foot_r_x[0]);
    foot_r->SetVelocity(foot_r_x[1]);
    foot_r->SetAcceleration(foot_r_x[2]);

    // Set input data to generate the expressions
    foot_l->SetInputs({sym_var["qacc"]}, {sym_var["qpos"], sym_var["qvel"]});
    foot_r->SetInputs({sym_var["qacc"]}, {sym_var["qpos"], sym_var["qvel"]});

    // Generate the functions for evaluation (here we've code-generated
    // them)
    foot_l->GenerateFunction("foot_l", true, "./");
    foot_r->GenerateFunction("foot_r", true, "./");

    // Add end-effectors to our vector
    ee_.push_back(foot_l);
    ee_.push_back(foot_r);

    /**
     * Tracking Tasks
     *
     */

    // Create tracking data for each foot
    osc::TrackingTaskDataFactory tracking_factory;
    // Create tasks
    tracking_tasks_["foot_l"] = tracking_factory.Create(
        foot_l, osc::TrackingTaskData::Type::kTranslational);
    tracking_tasks_["foot_r"] = tracking_factory.Create(
        foot_r, osc::TrackingTaskData::Type::kTranslational);

    // Add tracking costs
    for (auto &task : tracking_tasks_) {
        // Get task name
        std::string name = task.first;
        // Get the end-effector associated with the task
        osc::EndEffector &ee = task.second->GetEndEffector();

        // Create parameter for program for task-acceleration error
        // minimisation
        // TODO - Include task dimension in damotion library
        Eigen::Ref<const Eigen::MatrixXd> xaccd =
            program_.AddParameters(name + "_xaccd", 3);

        // Set tracking gains for the end-effectors
        task.second->Kp.diagonal().setConstant(2e2);
        task.second->Kd.diagonal().setConstant(1e0);

        // Create objective for tracking
        casadi::SX sym_xaccd = casadi::SX::sym("xacc_d", 3);

        // Create objective as || xacc_d - xacc ||^2
        sym::Expression obj =
            1e1 *
            casadi::SX::dot(ee.Acceleration()(casadi::Slice(0, 3)) - sym_xaccd,
                            ee.Acceleration()(casadi::Slice(0, 3)) - sym_xaccd);
        // Set inputs to the expression
        obj.SetInputs({sym_var["qacc"]},
                      {sym_var["qpos"], sym_var["qvel"], sym_xaccd});

        // Add objective to program
        std::shared_ptr<opt::QuadraticCost> task_cost =
            std::make_shared<opt::QuadraticCost>(name + "_tracking", obj);
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
    casadi::SX dJcldt = jacobian(mtimes(Jcl, sym_var["qvel"]), sym_var["qpos"]);

    // Constraint first derivative in time
    casadi::SX dc = mtimes(Jcl, sym_var["qvel"]);
    // Constraint second derivative in time
    casadi::SX d2c =
        mtimes(Jcl, sym_var["qacc"]) + mtimes(dJcldt, sym_var["qvel"]);

    closed_loop.SetConstraint(cl);
    closed_loop.SetConstraintFirstDerivative(dc);
    closed_loop.SetConstraintSecondDerivative(d2c);

    // Add constraint forces as decision variables
    sym::VariableVector lam_closed_loop =
        sym::CreateVariableVector("lam_closed_loop", cl.size1());
    program_.AddDecisionVariables(lam_closed_loop);
    // Add bounds to constraint forces
    program_.AddBoundingBoxConstraint(-1e3, 1e3, lam_closed_loop);

    constraints_.push_back(closed_loop);
    constraint_forces_.push_back(lam_closed_loop);

    // Convert to linear constraint in qacc
    casadi::SX A, b;
    casadi::SX::linear_coeff(d2c, sym_var["qacc"], A, b, true);
    auto closed_loop_con = std::make_shared<opt::LinearConstraint>(
        "closed_loop", A, b,
        casadi::SXVector({sym_var["qpos"], sym_var["qvel"]}),
        opt::BoundsType::kEquality);

    // Add linear constraint to program, binding variables and constraints
    // to it
    program_.AddLinearConstraint(closed_loop_con, {qacc_}, {qpos, qvel});

    std::cout << "Constraints\n";

    /**
     *
     *  Constrained dynamics
     *
     */
    casadi::SXVector in = {sym_var["qacc"], sym_var["ctrl"]};
    sym::VariableVector in_dyn(qacc_.size() + ctrl_.size());
    in_dyn << qacc_, ctrl_;

    // For each holonomic constraint
    for (int i = 0; i < constraints_.size(); i++) {
        // Add constraint forces to the dynamic expression input
        in_dyn.conservativeResize(in_dyn.size() + constraint_forces_[i].size());
        in_dyn.bottomRows(constraint_forces_[i].size())
            << constraint_forces_[i];
        // Create symbolic representation of the constraint forces
        casadi::SX lam = casadi::SX::sym("lam", constraint_forces_[i].size());
        in.push_back(lam);
        // Get constraint Jacobian
        casadi::SX J = jacobian(constraints_[i].ConstraintFirstDerivative(),
                                sym_var["qvel"]);
        // Add joint-space forces based on the constraints
        dynamics -= mtimes(J.T(), lam);
    }

    std::cout << "Dynamics\n";

    // Create single vector for linear expression
    // Set inputs for expression
    dynamics.SetInputs({vertcat(in)}, {sym_var["qpos"], sym_var["qvel"]});
    // Add constraint with variable bindings
    casadi::SX::linear_coeff(dynamics, vertcat(in), A, b, true);
    std::shared_ptr<opt::LinearConstraint> dynamics_con =
        std::make_shared<opt::LinearConstraint>("dynamics", A, b,
                                                dynamics.Parameters(),
                                                opt::BoundsType::kEquality);
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
        std::make_shared<opt::QuadraticCost>("torque_cost", u2);
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
        x.middleRows(idx, constraint_forces_[i].size()) = constraint_forces_[i];
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
    Eigen::VectorXd ctrl_max(CASSIE_FIXED_NU);
    ctrl_max << 4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2, 0.9;
    program_.AddBoundingBoxConstraint(-ctrl_max, ctrl_max, ctrl_);

    std::cout << "Bounds\n";

    // Show the program details
    program_.PrintProgramSummary();
    // Print detailed list of variables and constraints
    program_.ListDecisionVariables();
    program_.ListParameters();
    program_.ListCosts();
    program_.ListConstraints();

    // Create solver for the program
    solver_ = std::make_unique<opt::solvers::QPOASESSolverInstance>(program_);
}

void CassieFixedOSC::UpdateReferences(double time) {
    double l_phase = -(2.0 * M_PI / 4.0) * time;

    // Update all end-effector state estimates
    for (std::shared_ptr<osc::EndEffector> &e : ee_) {
        e->Function().setInput(1, qpos_.data());
        e->Function().setInput(2, qvel_.data());
        e->Function().call();
    }

    // Create tracking references
    osc::EndEffector &foot_l = tracking_tasks_["foot_l"]->GetEndEffector();
    osc::EndEffector &foot_r = tracking_tasks_["foot_r"]->GetEndEffector();
    // Create references
    Eigen::Vector3d &xl = tracking_tasks_["foot_l"]->xr,
                    &xr = tracking_tasks_["foot_r"]->xr;

    std::cout << "Left: " << foot_l.EvalPosition().topRows(3).transpose()
              << std::endl;
    std::cout << "Right: " << foot_r.EvalPosition().topRows(3).transpose()
              << std::endl;

    // Create trajectories
    xl[0] = 0.0 + 0.2 * cos(l_phase);
    xl[1] = 0.1;
    xl[2] = -0.7 + 0.2 * sin(l_phase);

    xr[0] = 0.0 + 0.2 * cos(l_phase);
    xr[1] = -0.1;
    xr[2] = -0.7 + 0.2 * sin(l_phase);

    std::cout << "Left Desired: " << xl.transpose() << std::endl;
    std::cout << "Left Error: "
              << tracking_tasks_["foot_l"]->GetPDError().transpose()
              << std::endl;
    std::cout << "Right Desired: " << xr.transpose() << std::endl;
    std::cout << "Right Error: "
              << tracking_tasks_["foot_r"]->GetPDError().transpose()
              << std::endl;

    tracking_tasks_["foot_l"]->qr = osc::RPYToQuaterion(0, 0, 0);
    tracking_tasks_["foot_r"]->qr = osc::RPYToQuaterion(0, 0, 0);

    // Compute new tracking errors
    program_.SetParameters("foot_l_xaccd",
                           tracking_tasks_["foot_l"]->GetPDError());
    program_.SetParameters("foot_r_xaccd",
                           tracking_tasks_["foot_r"]->GetPDError());
}

void CassieFixedOSC::UpdateState(int nq, const double *q, int nv,
                                 const double *v) {
    // Map Mujoco data to appropriate joints
    qpos_ << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13], q[14], q[15],
        q[16], q[21], q[22], q[23], q[24], q[27];
    qvel_ << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12], v[13], v[14],
        v[15], v[19], v[20], v[21], v[22], v[25];

    program_.SetParameters("qpos", qpos_);
    program_.SetParameters("qvel", qvel_);
}