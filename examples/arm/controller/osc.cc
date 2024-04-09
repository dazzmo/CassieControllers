#include "osc.h"

ArmOSC::ArmOSC() {
    // Load cassie model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("./arm.urdf", model, true);
    pinocchio::Data data(model);
    // Wrap model for casadi functionality
    damotion::utils::casadi::PinocchioModelWrapper wrapper(model);

    // Create symbolic variables for constraint/cost creation
    std::unordered_map<std::string, casadi::SX> sym_var;
    sym_var["qpos"] = casadi::SX::sym("qpos", model.nq);
    sym_var["qvel"] = casadi::SX::sym("qvel", model.nv);
    sym_var["qacc"] = casadi::SX::sym("qacc", model.nv);
    sym_var["ctrl"] = casadi::SX::sym("ctrl", ARM_NU);

    qpos_ = Eigen::VectorXd(model.nq);
    qvel_ = Eigen::VectorXd(model.nq);

    // Create program variables
    qacc_ = sym::CreateVariableVector("qacc", model.nv),
    ctrl_ = sym::CreateVariableVector("ctrl", ARM_NU);

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
    casadi::SX B = ArmActuationMatrix(wrapper.model(), wrapper.data(),
                                      sym_var["qpos"], sym_var["qvel"]);

    // Compute inverse dynamics M qacc + C qvel + G
    sym::Expression dynamics = wrapper.rnea()(casadi::SXVector(
        {sym_var["qpos"], sym_var["qvel"], sym_var["qacc"]}))[0];
    // Add inputs and other features
    dynamics -= mtimes(B, sym_var["ctrl"]);

    /**
     *
     * Register End-Effectors
     *
     */

    // Add any end-effectors of interest to the model
    wrapper.addEndEffector("hand");

    // Compute end-effector data for end-effectors
    casadi::SXVector ee_input =
        casadi::SXVector({sym_var["qpos"], sym_var["qvel"], sym_var["qacc"]});
    casadi::SXVector hand_x = wrapper.end_effector("hand").x(ee_input);

    // Create end-effectors
    osc::EndEffectorFactory ee_factory;
    std::shared_ptr<osc::EndEffector> hand = ee_factory.Create("hand");

    // Add expressions for the end-effectors
    hand->SetPosition(hand_x[0]);
    hand->SetVelocity(hand_x[1]);
    hand->SetAcceleration(hand_x[2]);

    // Set input data to generate the expressions
    hand->SetInputs({sym_var["qacc"]}, {sym_var["qpos"], sym_var["qvel"]});

    // Generate the functions for evaluation (here we've code-generated
    // them)
    hand->GenerateFunction("hand", true, "./");

    // Add end-effectors to our vector
    ee_.push_back(hand);

    /**
     * Tracking Tasks
     *
     */
    osc::TrackingTaskDataFactory tracking_factory;
    // Create positional tracking task
    tracking_tasks_["hand"] = tracking_factory.Create(
        hand, osc::TrackingTaskData::Type::kTranslational);

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
        task.second->Kp.diagonal().setConstant(1e2);
        task.second->Kd.diagonal().setConstant(1e1);

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

    std::cout << "Constraints\n";

    /**
     *
     *  Constrained dynamics
     *
     */
    casadi::SXVector in = {sym_var["qacc"], sym_var["ctrl"]};
    sym::VariableVector in_dyn(qacc_.size() + ctrl_.size());
    in_dyn << qacc_, ctrl_;

    std::cout << "Dynamics\n";

    // Set inputs for expression
    dynamics.SetInputs({vertcat(in)}, {sym_var["qpos"], sym_var["qvel"]});
    // Add constraint with variable bindings
    casadi::SX A, b;
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

    // Add a torque-regularisation cost
    sym::Expression u2 =
        1e-2 * casadi::SX::dot(sym_var["ctrl"], sym_var["ctrl"]);
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
    program_.AddBoundingBoxConstraint(-1e4, 1e4, qacc_);
    // Control inputs
    program_.AddBoundingBoxConstraint(-50.0, 50.0, ctrl_);

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

void ArmOSC::UpdateReferences(double time) {
    double l_phase = -(2.0 * M_PI / 4.0) * time;

    // Update all end-effector state estimates
    for (std::shared_ptr<osc::EndEffector> &e : ee_) {
        e->Function().setInput(1, qpos_.data());
        e->Function().setInput(2, qvel_.data());
        e->Function().call();
    }

    // Create tracking references
    osc::EndEffector &hand = tracking_tasks_["hand"]->GetEndEffector();
    // Create references
    Eigen::Vector3d &x = tracking_tasks_["hand"]->xr;
    std::cout << "Hand: " << hand.EvalPosition().topRows(3).transpose()
              << std::endl;

    // Create trajectories
    x[0] = 0.0;
    x[1] = 0.0 + 0.2 * cos(l_phase);
    x[2] = -1.5 + 0.2 * sin(l_phase);

    std::cout << "Desired: " << x.transpose() << std::endl;
    std::cout << "Error: " << tracking_tasks_["hand"]->GetPDError().transpose()
              << std::endl;

    // Compute new tracking errors
    program_.SetParameters("hand_xaccd", -tracking_tasks_["hand"]->GetPDError());
    program_.ListParameters();
}

void ArmOSC::UpdateState(int nq, const double *q, int nv, const double *v) {
    // Map Mujoco data to appropriate joints
    qpos_ << q[0], q[1], q[2];
    qvel_ << v[0], v[1], v[2];

    program_.SetParameters("qpos", qpos_);
    program_.SetParameters("qvel", qvel_);
}