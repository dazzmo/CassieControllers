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
    dynamics += (spring_forces + damping) - mtimes(B, sym_terms.ctrl());

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
    auto com = wrapper.com();

    foot_lf->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_lb->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_rf->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    foot_rb->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    pelvis->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());
    com->UpdateState(sym_terms.qpos(), sym_terms.qvel(), sym_terms.qacc());

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
    pelvis_->SetWeighting(1e2);

    com_ = std::make_shared<osc::PositionTask>("com", com);
    com_->SetWeighting(1e1);

    // Contact normals
    foot_lf_->SetWeighting(1e0);
    foot_lb_->SetWeighting(1e0);
    foot_rf_->SetWeighting(1e0);
    foot_rb_->SetWeighting(1e0);

    foot_lf_->SetNormal(Eigen::Vector3d::UnitZ());
    foot_lb_->SetNormal(Eigen::Vector3d::UnitZ());
    foot_rf_->SetNormal(Eigen::Vector3d::UnitZ());
    foot_rb_->SetNormal(Eigen::Vector3d::UnitZ());

    // Set default friction constraints
    foot_lf_->SetFrictionForceLimits(Eigen::Vector3d(-1e6, -1e6, 0.0),
                                     Eigen::Vector3d(1e6, 1e6, 1e6));
    foot_lb_->SetFrictionForceLimits(Eigen::Vector3d(-1e6, -1e6, 0.0),
                                     Eigen::Vector3d(1e6, 1e6, 1e6));
    foot_rf_->SetFrictionForceLimits(Eigen::Vector3d(-1e6, -1e6, 0.0),
                                     Eigen::Vector3d(1e6, 1e6, 1e6));
    foot_rb_->SetFrictionForceLimits(Eigen::Vector3d(-1e6, -1e6, 0.0),
                                     Eigen::Vector3d(1e6, 1e6, 1e6));

    // Set default references for foot locations
    foot_lf_->SetReference(Eigen::Vector3d(0.08, 0.135, 0));
    foot_lb_->SetReference(Eigen::Vector3d(-0.08, 0.135, 0));
    foot_rf_->SetReference(Eigen::Vector3d(0.08, -0.135, 0));
    foot_rb_->SetReference(Eigen::Vector3d(-0.08, -0.135, 0));

    // Set gains for PD tracking
    foot_lf_->SetKpGains(Eigen::Vector3d(5e1, 5e1, 5e1));
    foot_lb_->SetKpGains(Eigen::Vector3d(5e1, 5e1, 5e1));
    foot_rf_->SetKpGains(Eigen::Vector3d(5e1, 5e1, 5e1));
    foot_rb_->SetKpGains(Eigen::Vector3d(5e1, 5e1, 5e1));

    foot_lf_->SetKdGains(Eigen::Vector3d(5e0, 5e0, 5e0));
    foot_lb_->SetKdGains(Eigen::Vector3d(5e0, 5e0, 5e0));
    foot_rf_->SetKdGains(Eigen::Vector3d(5e0, 5e0, 5e0));
    foot_rb_->SetKdGains(Eigen::Vector3d(5e0, 5e0, 5e0));

    pelvis_->SetKpGains(Eigen::Vector3d(5e1, 5e1, 5e1));
    pelvis_->SetKdGains(Eigen::Vector3d(10, 10, 10));

    com_->SetKpGains(Eigen::Vector3d(1e2, 1e2, 1e2));
    com_->SetKdGains(Eigen::Vector3d(10, 10, 10));

    // Add motion tasks to OSC
    osc_->AddContactPoint(foot_lf_);
    osc_->AddContactPoint(foot_lb_);
    osc_->AddContactPoint(foot_rf_);
    osc_->AddContactPoint(foot_rb_);
    osc_->AddMotionTask(pelvis_);
    osc_->AddMotionTask(com_);

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
    casadi::SX a_joints = sym_terms.qacc()(casadi::Slice(6, 22));

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
    casadi::SX damping_task_error = a_joints - 1.0 * v_joints;
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
    sym::Expression u2 = 1e-5 * casadi::SX::dot(u_normalised, u_normalised);
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
    foot_lf_->inContact = true;
    foot_lb_->inContact = true;
    foot_rf_->inContact = true;
    foot_rb_->inContact = true;

    foot_lf_->Frame().UpdateState(qpos_, qvel_,
                                  Eigen::VectorXd::Zero(CASSIE_NV));
    foot_lb_->Frame().UpdateState(qpos_, qvel_,
                                  Eigen::VectorXd::Zero(CASSIE_NV));
    foot_rf_->Frame().UpdateState(qpos_, qvel_,
                                  Eigen::VectorXd::Zero(CASSIE_NV));
    foot_rb_->Frame().UpdateState(qpos_, qvel_,
                                  Eigen::VectorXd::Zero(CASSIE_NV));

    pelvis_->Frame().UpdateState(qpos_, qvel_,
                                 Eigen::VectorXd::Zero(CASSIE_NV));

    com_->Frame().UpdateState(qpos_, qvel_, Eigen::VectorXd::Zero(CASSIE_NV));

    // We can update a contact point by addressing the program
    // e.g. osc_->UpdateContactPoint(foot_lf_);

    std::cout << "foot_lf_ = " << foot_lf_->pos().transpose() << std::endl;
    std::cout << "foot_lb_ = " << foot_lb_->pos().transpose() << std::endl;
    std::cout << "foot_rf_ = " << foot_rf_->pos().transpose() << std::endl;
    std::cout << "foot_rb_ = " << foot_rb_->pos().transpose() << std::endl;
    std::cout << "CoM = " << com_->pos().transpose() << std::endl;

    // Lambda
    for (int i = 0; i < 4; i++) {
        std::cout << solver_->GetVariableValues(osc_->GetVariables().lambda(i))
                         .transpose()
                  << std::endl;
    }

    std::cout << "Pelvis: " << pelvis_->pos().transpose() << std::endl;
    std::cout << "Pelvis Desired: " << osc::RPYToQuaterion(0, 0, 0) << std::endl;
    std::cout << "Pelvis Error: " << pelvis_->Error().transpose() << std::endl;
    std::cout << "Pelvis Error Derivative: " << pelvis_->ErrorDerivative().transpose() << std::endl;

    pelvis_->SetReference(osc::RPYToQuaterion(0.6 * sin((2.0 * M_PI / 0.7) * time + 0.2), 0.0, 0.8 * sin((2.0 * M_PI / 0.5) * time)),
                          Eigen::Vector3d::Zero());

    com_->SetReference(Eigen::Vector3d(-0.02, 0.0, 0.8 + 0.05 * sin((2.0 * M_PI / 3.0) * time)),
                       Eigen::Vector3d::Zero());
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