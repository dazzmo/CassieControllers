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
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-M_PI_2, Eigen::Vector3d::UnitZ());
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

    // Get model and data references
    CodeGenerator::ADModel& model = cg.GetModel();
    CodeGenerator::ADData& data = cg.GetData();

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
    // Currently-used values from the cassie.xml MuJoCo model (but added small nonzero for heel)
    // Values will depend on which springs are currently attached to Cassie
    CodeGenerator::TangentVectorAD spring_forces = CodeGenerator::TangentVectorAD::Zero(model.nv);
    double k_knee_stiffness = 1500; //2300.0;
    double k_heel_stiffness = 1250; //2000.0;
    double b_knee_damping = 0.1;    //4.6;
    double b_heel_damping = 0.001;  //4.0;

    // Extract joint coordinates of springs
    casadi::SX q_knee = cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch"));
    casadi::SX q_heel = cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring"));

    casadi::SX v_knee = cg.GetQvelSX()(cg.GetJointIdv("LeftShinPitch"));
    casadi::SX v_heel = cg.GetQvelSX()(cg.GetJointIdv("LeftAchillesSpring"));

    // Add to spring forces
    spring_forces(cg.GetJointIdv("LeftShinPitch")) = k_knee_stiffness * (q_knee) + b_knee_damping * (v_knee);
    spring_forces(cg.GetJointIdv("LeftAchillesSpring")) = k_heel_stiffness * (q_heel) + b_heel_damping * (v_heel);

    // Add spring forces to bias vector
    h += spring_forces; // TODO: Subtract?

    // Damping forces
    CodeGenerator::TangentVectorAD damping(model.nv);
    damping(cg.GetJointIdv("LeftHipRoll")) = 10.0;
    damping(cg.GetJointIdv("LeftHipYaw")) = 1.0;
    damping(cg.GetJointIdv("LeftHipPitch")) = 1.0;
    damping(cg.GetJointIdv("LeftKneePitch")) = 1.0;
    damping(cg.GetJointIdv("LeftShinPitch")) = 0.1;
    damping(cg.GetJointIdv("LeftTarsusPitch")) = 0.1;
    damping(cg.GetJointIdv("LeftAchillesSpring")) = 0.0;
    damping(cg.GetJointIdv("LeftFootPitch")) = 1.0;

    // Add damping forces to bias vector
    h += damping; // TODO: Subtract?

    // Generate code for bias vector
    casadi::SX h_sx;
    pinocchio::casadi::copy(h, h_sx);
    cg.GenerateCode("bias_vector", {cg.GetQposSX(), cg.GetQvelSX()}, {h_sx});

    // Actuation matrix (filled with gear ratios)
    casadi::SX B = casadi::SX::zeros(model.nv, 5);
    B(cg.GetJointIdv("LeftHipRoll"), 0) = model.rotorGearRatio[cg.GetJointIdv("LeftHipRoll")];
    B(cg.GetJointIdv("LeftHipYaw"), 1) = model.rotorGearRatio[cg.GetJointIdv("LeftHipYaw")];
    B(cg.GetJointIdv("LeftHipPitch"), 2) = model.rotorGearRatio[cg.GetJointIdv("LeftHipPitch")];
    B(cg.GetJointIdv("LeftKneePitch"), 3) = model.rotorGearRatio[cg.GetJointIdv("LeftKneePitch")];
    B(cg.GetJointIdv("LeftFootPitch"), 4) = model.rotorGearRatio[cg.GetJointIdv("LeftFootPitch")];

    cg.GenerateCode("actuation_map", {cg.GetQposSX()}, {B});

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
