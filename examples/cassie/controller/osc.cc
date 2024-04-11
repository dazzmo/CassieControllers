#include "osc.h"

CassieOSC::CassieOSC() {
    // Load cassie model
    pinocchio::Model model;
    pinocchio::urdf::buildModel("./cassie.urdf", model);
    pinocchio::Data data(model);
    // Wrap model for casadi functionality
    damotion::utils::casadi::PinocchioModelWrapper wrapper(model);

    // Create conventional OSC program
    osc_ = std::make_unique<osc::OSC>(model.nq, model.nv, CASSIE_NU);
    std::cout << "OSC made\n";

    qpos_ = Eigen::VectorXd(model.nq);
    qvel_ = Eigen::VectorXd(model.nv);

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
    casadi::SX pd_joint_out = mtimes(pinv(B), 10.0 * sym_terms.qvel());
    // Create a function that generates the output
    casadi::Function pd_out("pd_out", {sym_terms.qvel()}, {pd_joint_out});

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
    auto foot_lf = wrapper.AddEndEffector("LeftFootFront");
    auto foot_lb = wrapper.AddEndEffector("LeftFootBack");
    auto foot_rf = wrapper.AddEndEffector("RightFootFront");
    auto foot_rb = wrapper.AddEndEffector("RightFootBack");
    auto pelvis = wrapper.AddEndEffector("base_joint");
    // auto com = wrapper.com();

    foot_lf->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_lb->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_rf->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_rb->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    pelvis->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());

    /**
     * Motion Tasks
     *
     */

    // Create tasks
    foot_lf_ = std::make_shared<osc::ContactTask3D>("foot_lf", foot_lf);
    foot_lb_ = std::make_shared<osc::ContactTask3D>("foot_lb", foot_lb);
    foot_rf_ = std::make_shared<osc::ContactTask3D>("foot_rf", foot_rf);
    foot_rb_ = std::make_shared<osc::ContactTask3D>("foot_rb", foot_rb);

    pelvis_ = std::make_shared<osc::OrientationTask>("pelvis", pelvis);

    // Set default friction constraints
    foot_lf_->SetFrictionForceLimits(Eigen::Vector3d(-1e3, -1e3, 0.0),
                                     Eigen::Vector3d(1e3, 1e3, 1e3));
    foot_lb_->SetFrictionForceLimits(Eigen::Vector3d(-1e3, -1e3, 0.0),
                                     Eigen::Vector3d(1e3, 1e3, 1e3));
    foot_rf_->SetFrictionForceLimits(Eigen::Vector3d(-1e3, -1e3, 0.0),
                                     Eigen::Vector3d(1e3, 1e3, 1e3));
    foot_rb_->SetFrictionForceLimits(Eigen::Vector3d(-1e3, -1e3, 0.0),
                                     Eigen::Vector3d(1e3, 1e3, 1e3));

    // Set default references for foot locations
    foot_lf_->SetReference(Eigen::Vector3d(0.08, 0.135, 0));
    foot_lb_->SetReference(Eigen::Vector3d(-0.08, 0.135, 0));
    foot_rf_->SetReference(Eigen::Vector3d(0.08, -0.135, 0));
    foot_rb_->SetReference(Eigen::Vector3d(-0.08, -0.135, 0));

    // Set gains
    // TODO

    // Add motion tasks to OSC
    osc_->AddContactPoint(foot_lf_);
    osc_->AddContactPoint(foot_lb_);
    osc_->AddContactPoint(foot_rf_);
    osc_->AddContactPoint(foot_rb_);
    osc_->AddMotionTask(pelvis_);

    std::cout << "Motion tasks\n";

    /**
     * Holonomic Constraints
     *
     */

    casadi::SX cl = CassieClosedLoopConstraint(
        wrapper.model(), wrapper.data(), sym_terms.qpos(), sym_terms.qvel());

    // Get Jacobian wrt to joints only
    casadi::SX q_joints = sym_terms.qpos()(casadi::Slice(7, 23));
    casadi::SX v_joints = sym_terms.qvel()(casadi::Slice(6, 22));

    // Compute first and second derivatives
    casadi::SX Jcl = jacobian(cl, q_joints);
    casadi::SX dJcldt = jacobian(mtimes(Jcl, v_joints), q_joints);

    // Pad zeros at the front to account for the floating base
    Jcl = casadi::SX::horzcat({casadi::SX(cl.size1(), 6), Jcl});
    dJcldt = casadi::SX::horzcat({casadi::SX(cl.size1(), 6), dJcldt});

    // Constraint first derivative in time
    casadi::SX dc = mtimes(Jcl, sym_terms.qvel());
    // Constraint second derivative in time
    casadi::SX d2c =
        mtimes(Jcl, sym_terms.qacc()) + mtimes(dJcldt, sym_terms.qvel());

    osc_->AddHolonomicConstraint("closed_loop", cl, dc, d2c);
    // Add bounds to constraint forces
    osc_->GetProgram().AddBoundingBoxConstraint(
        -1e6, 1e6, osc_->GetVariables().lambda().back());

    std::cout << "Constraints\n";

    /**
     *
     * Other Costs
     *
     */

    // Joint damping
    casadi::SX damping_task_error = sym_terms.qacc() - 1.0 * sym_terms.qvel();
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
    Eigen::VectorXd ctrl_max(CASSIE_NU);
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

void CassieOSC::UpdateReferences(double time) {
    // Update all task states
    foot_lf_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(CASSIE_NV)); 
    foot_lb_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(CASSIE_NV)); 
    foot_rf_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(CASSIE_NV)); 
    foot_rb_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(CASSIE_NV)); 

    // com_->SetReference(Eigen::Vector3d(
    //     -0.01656, 0.0, 0.75 - 0.1 * sin(2 * M_PI * time / 2.0)));

    // Eigen::Vector3d rl, rr;
    // rl[0] = 0.0 + 0.2 * cos(l_phase);
    // rl[1] = 0.1;
    // rl[2] = -0.8 + 0.2 * sin(l_phase);

    // rr[0] = 0.0 + 0.2 * cos(l_phase);
    // rr[1] = -0.1;
    // rr[2] = -0.8 + 0.2 * sin(l_phase);

    // // Update frame placements
    // left_foot_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(16));
    // right_foot_->Frame().UpdateState(qpos_, qvel_,
    // Eigen::VectorXd::Zero(16));

    // left_foot_->SetReference(rl, Eigen::Vector3d::Zero());
    // right_foot_->SetReference(rr, Eigen::Vector3d::Zero());
    // // TODO - Look up what the correct orientation for the foot is to keep it
    // // level
    // right_foot_orientation_->SetReference(osc::RPYToQuaterion(0, 0, 0),
    //                                       Eigen::Vector3d::Zero());
    // std::cout << right_foot_orientation_->GetReference().q << std::endl;
    // std::cout << right_foot_orientation_->Frame().pos() << std::endl;
    // std::cout << right_foot_orientation_->Frame().vel() << std::endl;
}

void CassieOSC::UpdateState(int nq, const double *q, int nv, const double *v) {
    // Pinocchio currently has quaternions as q = [x, y, z, w]
    qpos_ << q[0], q[1], q[2], q[4], q[5], q[6], q[3], q[7], q[8], q[9], q[14],
        q[15], q[16], q[17], q[20], q[21], q[22], q[23], q[28], q[29], q[30],
        q[31], q[34];

    qvel_ << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[12], v[13],
        v[14], v[15], v[18], v[19], v[20], v[21], v[25], v[26], v[27], v[28],
        v[31];
}