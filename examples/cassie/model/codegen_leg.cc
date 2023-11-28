#include "code_generator.h"

// TODO: Read all the constants from a .yaml file
// TODO: Provide better estimate of damping terms? (see other Cassie groups)

int main(int argc, char* argv[]) {

    // Initialise model from Cassie urdf (just one leg)
    CodeGenerator cg("./cassie_leg.urdf");
    cg.SetCodeGenerationDestination(argv[1]);

    /********** Create reference frames **********/

    // TODO: Double-check the order (ZYX vs. XYZ)
    // Sole of foot frame (https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model)
    Eigen::AngleAxisd rollAngle(-M_PI_2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond foot_rot_q = yawAngle * pitchAngle * rollAngle;

    cg.AddReferenceFrame(
        "foot",
        "LeftFootPitch",
        "leftfoot",
        Eigen::Vector3d(0.01762, 0.05219, 0.0),
        foot_rot_q.matrix());

    // https://github.com/agilityrobotics/cassie-doc/wiki/Heel-Spring-Model
    cg.AddReferenceFrame(
        "heel_tip",
        "LeftAchillesSpring",
        "leftheelspring",
        Eigen::Vector3d(0.11877, -0.01, 0.0),
        Eigen::Matrix3d::Identity());

    // https://github.com/agilityrobotics/cassie-doc/wiki/Thigh-Model
    cg.AddReferenceFrame(
        "achilles_socket",
        "LeftHipPitch",
        "lefthippitch",
        Eigen::Vector3d(0.0, 0.0, 0.045),
        Eigen::Matrix3d::Identity());

    // Get model and data references, update forward kinematics
    CodeGenerator::ADModel& model = cg.GetModel();
    CodeGenerator::ADData& data = cg.GetData();
    pinocchio::framesForwardKinematics(model, data, cg.GetQpos());

    // Compute inertia matrix
    cg.GenerateInertiaMatrix();

    // Add gearing and rotor inertia information (all from Cassie wiki)
    model.rotorGearRatio[cg.GetJointIdv("LeftHipRoll")] = 25.0;
    model.rotorGearRatio[cg.GetJointIdv("LeftHipYaw")] = 25.0;
    model.rotorGearRatio[cg.GetJointIdv("LeftHipPitch")] = 16.0;
    model.rotorGearRatio[cg.GetJointIdv("LeftKneePitch")] = 16.0;
    model.rotorGearRatio[cg.GetJointIdv("LeftFootPitch")] = 50.0;

    model.rotorInertia[cg.GetJointIdv("LeftHipRoll")] = 6.10e-05;
    model.rotorInertia[cg.GetJointIdv("LeftHipYaw")] = 6.10e-05;
    model.rotorInertia[cg.GetJointIdv("LeftHipPitch")] = 3.65e-04;
    model.rotorInertia[cg.GetJointIdv("LeftKneePitch")] = 3.65e-04;
    model.rotorInertia[cg.GetJointIdv("LeftFootPitch")] = 4.90e-06;

    // Achilles rod on Cassie constrains each end to be 501.2mm apart:
    // https://github.com/agilityrobotics/cassie-doc/wiki/Achilles-Rod-Model
    const double achilles_length = 0.5012;

    // Create constraint
    CodeGenerator::ADData::Vector3 dl = data.oMf[model.getFrameId("achilles_socket")].translation() -
                                        data.oMf[model.getFrameId("heel_tip")].translation();
    casadi::SX cl = dl.squaredNorm() - achilles_length * achilles_length;

    // // Uncomment for a stiff model TODO: Not finished/working yet
    // casadi::SX cl = casadi::SX::zeros(3);
    // cl(0) = cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring"));
    // cl(1) = cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch"));
    // cl(2) = cg.GetQposSX()(cg.GetJointIdq("LeftKneePitch")) -
    //         cg.GetQposSX()(cg.GetJointIdq("LeftTarsusPitch")) + 13 * M_PI / 180.0;

    // Get Jacobian of the closed-loop constraint, its time-derivative, and the Hessian
    // Note that dJdt = dJdq * dqdt by chain rule
    casadi::SX Jcl = jacobian(cl, cg.GetQposSX());
    casadi::SX Hcl = hessian(cl, cg.GetQposSX());
    casadi::SX dJcldt = jacobian(mtimes(Jcl, cg.GetQvelSX()), cg.GetQposSX());

    // Generate code for constraint
    cg.GenerateCode(
        "achilles_rod_constraint", 
        {cg.GetQposSX(), cg.GetQvelSX()},
        {densify(cl), densify(Jcl), densify(mtimes(dJcldt, cg.GetQvelSX())), densify(Hcl)}
    );

    // Bias vector (includes gravity, damping, spring forces, etc.)
    CodeGenerator::TangentVectorAD h = pinocchio::rnea<CodeGenerator::ADScalar>(
        model, 
        data,
        cg.GetQpos(),
        cg.GetQvel(),
        CodeGenerator::TangentVectorAD::Zero(model.nv)
    );

    // Spring dynamics parameters. Commented values from:
    // https://github.com/jpreher/cassie_description/blob/master/MATLAB/Cassie_v4.m#L193
    // Currently-used values from the cassie.xml MuJoCo model
    // Values will depend on which springs are currently attached to Cassie
    double k_knee_stiffness = 1500; //2300.0;
    double k_heel_stiffness = 1250; //2000.0;

    // Extract joint coordinates of springs
    casadi::SX q_knee = cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch"));
    casadi::SX q_heel = cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring"));

    // Add to spring forces
    CodeGenerator::TangentVectorAD spring_forces = CodeGenerator::TangentVectorAD::Zero(model.nv);
    spring_forces(cg.GetJointIdv("LeftShinPitch")) = k_knee_stiffness * (q_knee);
    spring_forces(cg.GetJointIdv("LeftAchillesSpring")) = k_heel_stiffness * (q_heel);

    // Add spring forces to bias vector
    h += spring_forces;

    // Damping forces (from MuJoCo model, cassie.xml)
    // Commented values from same source as springs
    double d_lhiproll     = 1.0;
    double d_lhipyaw      = 1.0;
    double d_lhippitch    = 1.0;
    double d_lknee        = 1.0;
    double d_lshinspring  = 0.1;     //4.6
    double d_ltarsus      = 0.1;
    double d_lheelspring  = 0.001;   //4.0
    double d_lfoot        = 1.0;

    // Add to damping forces
    // TODO: Could do this more nicely with a diagonal matrix
    CodeGenerator::TangentVectorAD damping(model.nv);
    damping(cg.GetJointIdv("LeftHipRoll"))        = d_lhiproll    * cg.GetQvelSX()(cg.GetJointIdv("LeftHipRoll"));
    damping(cg.GetJointIdv("LeftHipYaw"))         = d_lhipyaw     * cg.GetQvelSX()(cg.GetJointIdv("LeftHipYaw"));
    damping(cg.GetJointIdv("LeftHipPitch"))       = d_lhippitch   * cg.GetQvelSX()(cg.GetJointIdv("LeftHipPitch"));
    damping(cg.GetJointIdv("LeftKneePitch"))      = d_lknee       * cg.GetQvelSX()(cg.GetJointIdv("LeftKneePitch"));
    damping(cg.GetJointIdv("LeftShinPitch"))      = d_lshinspring * cg.GetQvelSX()(cg.GetJointIdv("LeftShinPitch"));
    damping(cg.GetJointIdv("LeftTarsusPitch"))    = d_ltarsus     * cg.GetQvelSX()(cg.GetJointIdv("LeftTarsusPitch"));
    damping(cg.GetJointIdv("LeftAchillesSpring")) = d_lheelspring * cg.GetQvelSX()(cg.GetJointIdv("LeftAchillesSpring"));
    damping(cg.GetJointIdv("LeftFootPitch"))      = d_lfoot       * cg.GetQvelSX()(cg.GetJointIdv("LeftFootPitch"));

    // Add damping forces to bias vector
    h += damping;

    // Generate code for bias vector
    casadi::SX h_sx;
    pinocchio::casadi::copy(h, h_sx);
    cg.GenerateCode("bias_vector", {cg.GetQposSX(), cg.GetQvelSX()}, {h_sx});

    // Actuation matrix (filled with gear ratios)
    casadi::SX B = casadi::SX::zeros(model.nv, 5);
    B(cg.GetJointIdv("LeftHipRoll"), 0)   = model.rotorGearRatio[cg.GetJointIdv("LeftHipRoll")];
    B(cg.GetJointIdv("LeftHipYaw"), 1)    = model.rotorGearRatio[cg.GetJointIdv("LeftHipYaw")];
    B(cg.GetJointIdv("LeftHipPitch"), 2)  = model.rotorGearRatio[cg.GetJointIdv("LeftHipPitch")];
    B(cg.GetJointIdv("LeftKneePitch"), 3) = model.rotorGearRatio[cg.GetJointIdv("LeftKneePitch")];
    B(cg.GetJointIdv("LeftFootPitch"), 4) = model.rotorGearRatio[cg.GetJointIdv("LeftFootPitch")];

    cg.GenerateCode("actuation_map", {cg.GetQposSX()}, {densify(B)});

    // Generate end-effector data
    // https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model
    cg.GenerateEndEffectorData("foot_front",
                               "LeftFootPitch", "foot",
                               Eigen::Vector3d(0.09, 0.0, 0.0),
                               Eigen::Matrix3d::Identity());

    cg.GenerateEndEffectorData("foot_back",
                               "LeftFootPitch", "foot",
                               Eigen::Vector3d(-0.09, 0.0, 0.0),
                               Eigen::Matrix3d::Identity());

    cg.GenerateEndEffectorData("ankle",
                               "LeftFootPitch", "leftfoot",
                               Eigen::Vector3d(0.0, 0.0, 0.0),
                               Eigen::Matrix3d::Identity());

    return 0;
}
