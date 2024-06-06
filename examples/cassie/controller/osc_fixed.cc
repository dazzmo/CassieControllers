#include "osc_fixed.h"

CassieFixedOSC::CassieFixedOSC() {
    // Load cassie model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("./cassie_fixed.urdf", model);
    pinocchio::Data data(model);
    // Wrap model for casadi functionality
    damotion::utils::casadi::PinocchioModelWrapper wrapper(model);

    // Create conventional OSC program
    osc_ = std::make_unique<osc::OSC>(model.nq, model.nv, CASSIE_FIXED_NU);
    std::cout << "OSC made\n";

    qpos_ = Eigen::VectorXd(model.nq);
    qvel_ = Eigen::VectorXd(model.nq);

    // Get symbolic variables for reference
    osc::OSC::SymbolicTerms &sym_terms = osc_->GetSymbolicTerms();

    /**
     * Unconstrained Dynamics
     *
     */
    // Get actuation matrix for Cassie
    casadi::SX B = CassieActuationMatrix(wrapper.model(), wrapper.data(),
                                         sym_terms.qpos(), sym_terms.qvel());

    // Create a joint-damping output PD term
    // casadi::SX pd_joint_out = mtimes(pinv(B), 10.0 * sym_terms.qvel());
    // Create a function that generates the output
    // casadi::Function pd_out("pd_out", {sym_terms.qvel()}, {pd_joint_out});

    // Add any additional nonlinearities (e.g. spring/damping of joints)
    casadi::SX spring_forces =
                   CassieSpringForces(wrapper.model(), wrapper.data(),
                                      sym_terms.qpos(), sym_terms.qvel()),
               damping =
                   CassieJointDampingForces(wrapper.model(), wrapper.data(),
                                            sym_terms.qpos(), sym_terms.qvel());

    // Compute inverse dynamics M qacc + C qvel + G
    sym::Expression dynamics = wrapper.rnea()(casadi::SXVector(
        {sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc()}))[0];
    // Add inputs and other features
    dynamics -= mtimes(B, sym_terms.ctrl());
    dynamics += (spring_forces + damping);

    // Add dynamics to the OSC program
    osc_->AddUnconstrainedInverseDynamics(dynamics);

    std::cout << "Dynamics made\n";

    /** Register End-Effectors **/

    // Add any end-effectors of interest to the model
    auto foot_l = wrapper.AddEndEffector("LeftFootFront");
    auto foot_r = wrapper.AddEndEffector("RightFootFront");

    foot_l->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_r->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());

    /**
     * Motion Tasks
     *
     */

    // Create tasks
    left_foot_ = std::make_shared<osc::PositionTask>("foot_l", foot_l);
    right_foot_ = std::make_shared<osc::PositionTask>("foot_r", foot_r);
    // right_foot_orientation_ = std::make_shared<osc::OrientationTask>("foot_r_orientation", foot_r);
    
    // Set gains
    left_foot_->SetKpGains(Eigen::Vector3d(1e2, 1e2, 1e2));
    left_foot_->SetKdGains(Eigen::Vector3d(1e1, 1e1, 1e1));

    right_foot_->SetKpGains(Eigen::Vector3d(1e2, 1e2, 1e2));
    right_foot_->SetKdGains(Eigen::Vector3d(1e1, 1e1, 1e1));

    // right_foot_orientation_->SetKpGains(Eigen::Vector3d(1e2, 1e2, 1e2));
    // right_foot_orientation_->SetKdGains(Eigen::Vector3d(1e1, 1e1, 1e1));

    // Add motion tasks to OSC
    osc_->AddMotionTask(left_foot_);
    osc_->AddMotionTask(right_foot_);
    // osc_->AddMotionTask(right_foot_orientation_);


    std::cout << "Motion tasks\n";

    /**
     * Holonomic Constraints
     *
     */

    casadi::SX cl = CassieClosedLoopConstraint(
        wrapper.model(), wrapper.data(), sym_terms.qpos(), sym_terms.qvel());
    // Compute first and second derivatives
    casadi::SX Jcl = jacobian(cl, sym_terms.qpos());
    casadi::SX dJcldt =
        jacobian(mtimes(Jcl, sym_terms.qvel()), sym_terms.qpos());

    // Constraint first derivative in time
    casadi::SX dc = mtimes(Jcl, sym_terms.qvel());
    // Constraint second derivative in time
    casadi::SX d2c =
        mtimes(Jcl, sym_terms.qacc()) + mtimes(dJcldt, sym_terms.qvel());

    osc_->AddHolonomicConstraint("closed_loop", cl, dc, d2c);
    // Add bounds to constraint forces
    osc_->GetProgram().AddBoundingBoxConstraint(-1e6, 1e6,
                                                osc_->GetVariables().lambda(0));

    std::cout << "Constraints\n";

    /**
     *
     * Other Costs
     *
     */

    // Joint damping
    casadi::SX damping_task_error = sym_terms.qacc() + 100.0 * sym_terms.qvel();
    sym::Expression joint_damping =
        1e-3 * casadi::SX::dot(damping_task_error, damping_task_error);
    joint_damping.SetInputs({sym_terms.qacc()}, {sym_terms.qvel()});
    std::shared_ptr<opt::QuadraticCost> pdamping =
        std::make_shared<opt::QuadraticCost>("damping", joint_damping);
    osc_->GetProgram().AddQuadraticCost(
        pdamping, {osc_->GetVariables().qacc()},
        {osc_->GetProgram().GetParameters("qvel")});

    // Add a torque-regularisation cost
    // Create normalised torque outputs
    casadi::SX u_normalised = sym_terms.ctrl();
    u_normalised(0) /= 4.5;
    u_normalised(1) /= 4.5;
    u_normalised(2) /= 12.2;
    u_normalised(3) /= 12.2;
    u_normalised(4) /= 0.9;
    u_normalised(5) /= 4.5;
    u_normalised(6) /= 4.5;
    u_normalised(7) /= 12.2;
    u_normalised(8) /= 12.2;
    u_normalised(9) /= 0.9;
    sym::Expression u2 = 1e-4 * casadi::SX::dot(u_normalised, u_normalised);
    u2.SetInputs({sym_terms.ctrl()}, {});
    std::shared_ptr<opt::QuadraticCost> pu2 =
        std::make_shared<opt::QuadraticCost>("torque_cost", u2);
    osc_->GetProgram().AddQuadraticCost(pu2, {osc_->GetVariables().ctrl()}, {});

    std::cout << "Costs\n";

    /**
     * Bounds
     *
     */
    // Joint accelerations
    osc_->GetProgram().AddBoundingBoxConstraint(-1e4, 1e4,
                                                osc_->GetVariables().qacc());
    // Control inputs
    Eigen::VectorXd ctrl_max(CASSIE_FIXED_NU);
    ctrl_max << 4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2, 0.9;
    osc_->GetProgram().AddBoundingBoxConstraint(-ctrl_max, ctrl_max,
                                                osc_->GetVariables().ctrl());

    // Create the program
    osc_->CreateProgram();

    // Show the program details
    osc_->GetProgram().PrintProgramSummary();
    // Print detailed list of variables and constraints
    osc_->GetProgram().ListDecisionVariables();
    osc_->GetProgram().ListParameters();
    osc_->GetProgram().ListCosts();
    osc_->GetProgram().ListConstraints();

    // Create solver for the current OSC program
    solver_ = std::make_unique<opt::solvers::QPOASESSolverInstance>(
        osc_->GetProgram());
}

void CassieFixedOSC::UpdateReferences(double time) {
    double l_phase = (2.0 * M_PI / 4.0) * time;

    Eigen::Vector3d rl, rr;
    rl[0] = 0.0 + 0.2 * cos(l_phase);
    rl[1] = 0.1;
    rl[2] = -0.7 + 0.2 * sin(l_phase);

    rr[0] = 0.0 + 0.2 * cos(l_phase + M_PI_2);
    rr[1] = -0.1;
    rr[2] = -0.7 + 0.2 * sin(l_phase + M_PI_2);


    // Update frame placements
    left_foot_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(16));
    right_foot_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(16));

    left_foot_->SetReference(rl, Eigen::Vector3d::Zero());
    right_foot_->SetReference(rr, Eigen::Vector3d::Zero());
    // // TODO - Look up what the correct orientation for the foot is to keep it level
    // right_foot_orientation_->SetReference(osc::RPYToQuaterion(0, 0, 0), Eigen::Vector3d::Zero());
    // std::cout << right_foot_orientation_->GetReference().q  << std::endl;
    // std::cout << right_foot_orientation_->Frame().pos()  << std::endl;
    // std::cout << right_foot_orientation_->Frame().vel()  << std::endl;

}

void CassieFixedOSC::UpdateState(int nq, const double *q, int nv,
                                 const double *v) {
    // Map Mujoco data to appropriate joints
    qpos_ << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13], q[14], q[15],
        q[16], q[21], q[22], q[23], q[24], q[27];
    qvel_ << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12], v[13], v[14],
        v[15], v[19], v[20], v[21], v[22], v[25];
}