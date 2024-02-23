#include "code_generator.h"

// TODO: Read all the constants from a .yaml file
// TODO: Provide better estimate of damping terms? (see other Cassie groups)
// TODO: Add mass matrix to next version (Cassie off stand)

int main(int argc, char* argv[]) {

    // Initialise model from Cassie urdf
    CodeGenerator cg("./cassie_fixed.urdf");
    cg.SetCodeGenerationDestination(argv[1]);
    
    // Create useful reference frames
    // https://github.com/agilityrobotics/cassie-doc/wiki/Heel-Spring-Model
    cg.AddReferenceFrame("left_heel_spring_tip", "LeftAchillesSpring",
                         Eigen::Vector3d(0.11877, -0.01, 0.0), Eigen::Matrix3d::Identity());
    cg.AddReferenceFrame("right_heel_spring_tip", "RightAchillesSpring",
                         Eigen::Vector3d(0.11877, -0.01, 0.0), Eigen::Matrix3d::Identity());

    // https://github.com/agilityrobotics/cassie-doc/wiki/Thigh-Model
    cg.AddReferenceFrame("left_achilles_rod_socket", "LeftHipPitch", 
                         Eigen::Vector3d(0.0, 0.0, 0.045), Eigen::Matrix3d::Identity());
    cg.AddReferenceFrame("right_achilles_rod_socket", "RightHipPitch",
                         Eigen::Vector3d(0.0, 0.0, 0.045), Eigen::Matrix3d::Identity());

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

    model.rotorGearRatio[cg.GetJointIdv("RightHipRoll")] = 25.0;
    model.rotorGearRatio[cg.GetJointIdv("RightHipYaw")] = 25.0;
    model.rotorGearRatio[cg.GetJointIdv("RightHipPitch")] = 16.0;
    model.rotorGearRatio[cg.GetJointIdv("RightKneePitch")] = 16.0;
    model.rotorGearRatio[cg.GetJointIdv("RightFootPitch")] = 50.0;

    model.rotorInertia[cg.GetJointIdv("LeftHipRoll")] = 6.10e-05;
    model.rotorInertia[cg.GetJointIdv("LeftHipYaw")] = 6.10e-05;
    model.rotorInertia[cg.GetJointIdv("LeftHipPitch")] = 3.65e-04;
    model.rotorInertia[cg.GetJointIdv("LeftKneePitch")] = 3.65e-04;
    model.rotorInertia[cg.GetJointIdv("LeftFootPitch")] = 4.90e-06;

    model.rotorInertia[cg.GetJointIdv("RightHipRoll")] = 6.10e-05;
    model.rotorInertia[cg.GetJointIdv("RightHipYaw")] = 6.10e-05;
    model.rotorInertia[cg.GetJointIdv("RightHipPitch")] = 3.65e-04;
    model.rotorInertia[cg.GetJointIdv("RightKneePitch")] = 3.65e-04;
    model.rotorInertia[cg.GetJointIdv("RightFootPitch")] = 4.90e-06;

    std::cout << "Inertia model" << std::endl;


    // Achilles rod on Cassie constrains each end to be 501.2mm apart:
    // https://github.com/agilityrobotics/cassie-doc/wiki/Achilles-Rod-Model
    const double achilles_length = 0.5012;

    // Create constraints, including for leaf springs, which we set to 0 deflection
    CodeGenerator::ADData::Vector3 dl = data.oMf[model.getFrameId("left_achilles_rod_socket")].translation() -
                                        data.oMf[model.getFrameId("left_heel_spring_tip")].translation();
    CodeGenerator::ADData::Vector3 dr = data.oMf[model.getFrameId("right_achilles_rod_socket")].translation() -
                                        data.oMf[model.getFrameId("right_heel_spring_tip")].translation();

    std::cout << "Achilles disctances" << std::endl;


    casadi::SX cl = casadi::SX::vertcat({dl.squaredNorm() - achilles_length * achilles_length,
                                         dr.squaredNorm() - achilles_length * achilles_length,
                                         cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch")),
                                         cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring")),
                                         cg.GetQposSX()(cg.GetJointIdq("RightShinPitch")),
                                         cg.GetQposSX()(cg.GetJointIdq("RightAchillesSpring"))});

    // Get Jacobian and its time-derivative for the constraints
    // Note that dJdt = dJdq * dqdt by the chain rule
    casadi::SX Jcl = jacobian(cl, cg.GetQposSX());
    casadi::SX dJcldt = jacobian(mtimes(Jcl, cg.GetQvelSX()), cg.GetQposSX());

    // Generate code for the constraints
    cg.GenerateCode(
        "achilles_rod_constraints",
        {cg.GetQposSX(), cg.GetQvelSX()},
        {densify(cl), densify(Jcl), densify(mtimes(dJcldt, cg.GetQvelSX()))});

    // Bias vector (includes gravity, damping, spring forces, etc.)
    CodeGenerator::TangentVectorAD h = pinocchio::rnea<CodeGenerator::ADScalar>(
        model,
        data,
        cg.GetQpos(),
        cg.GetQvel(),
        CodeGenerator::TangentVectorAD::Zero(model.nv));

    // Spring dynamics parameters. Commented values from:
    // https://github.com/jpreher/cassie_description/blob/master/MATLAB/Cassie_v4.m#L193
    // Currently-used values from the cassie.xml MuJoCo model
    // Values will depend on which springs are currently attached to Cassie
    double knee_stiffness = 1500;  // 2300.0;
    double heel_stiffness = 1250;  // 2000.0;

    // Extract joint coordinates of springs
    casadi::SX q_l_knee = cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch"));
    casadi::SX q_r_knee = cg.GetQposSX()(cg.GetJointIdq("RightShinPitch"));
    casadi::SX q_l_heel = cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring"));
    casadi::SX q_r_heel = cg.GetQposSX()(cg.GetJointIdq("RightAchillesSpring"));

    // Add to spring forces
    CodeGenerator::TangentVectorAD spring_forces = CodeGenerator::TangentVectorAD::Zero(model.nv);
    spring_forces(cg.GetJointIdv("LeftShinPitch")) = knee_stiffness * (q_l_knee);
    spring_forces(cg.GetJointIdv("RightShinPitch")) = knee_stiffness * (q_r_knee);
    spring_forces(cg.GetJointIdv("LeftAchillesSpring")) = heel_stiffness * (q_l_heel);
    spring_forces(cg.GetJointIdv("RightAchillesSpring")) = heel_stiffness * (q_r_heel);

    // Add spring forces to bias vector
    h += spring_forces;

    // Damping forces (from MuJoCo model, cassie.xml)
    // Commented values from same source as springs
    double d_hiproll = 1.0;
    double d_hipyaw = 1.0;
    double d_hippitch = 1.0;
    double d_knee = 1.0;
    double d_shinspring = 0.1;    // 4.6
    double d_tarsus = 0.1;
    double d_heelspring = 0.001;  // 4.0
    double d_foot = 1.0;

    // Add to damping forces
    // TODO: Could do this more nicely with a diagonal matrix
    CodeGenerator::TangentVectorAD damping = CodeGenerator::TangentVectorAD::Zero(model.nv);
    damping(cg.GetJointIdv("LeftHipRoll")) = d_hiproll * cg.GetQvelSX()(cg.GetJointIdv("LeftHipRoll"));
    damping(cg.GetJointIdv("LeftHipYaw")) = d_hipyaw * cg.GetQvelSX()(cg.GetJointIdv("LeftHipYaw"));
    damping(cg.GetJointIdv("LeftHipPitch")) = d_hippitch * cg.GetQvelSX()(cg.GetJointIdv("LeftHipPitch"));
    damping(cg.GetJointIdv("LeftKneePitch")) = d_knee * cg.GetQvelSX()(cg.GetJointIdv("LeftKneePitch"));
    damping(cg.GetJointIdv("LeftShinPitch")) = d_shinspring * cg.GetQvelSX()(cg.GetJointIdv("LeftShinPitch"));
    damping(cg.GetJointIdv("LeftTarsusPitch")) = d_tarsus * cg.GetQvelSX()(cg.GetJointIdv("LeftTarsusPitch"));
    damping(cg.GetJointIdv("LeftAchillesSpring")) = d_heelspring * cg.GetQvelSX()(cg.GetJointIdv("LeftAchillesSpring"));
    damping(cg.GetJointIdv("LeftFootPitch")) = d_foot * cg.GetQvelSX()(cg.GetJointIdv("LeftFootPitch"));

    damping(cg.GetJointIdv("RightHipRoll")) = d_hiproll * cg.GetQvelSX()(cg.GetJointIdv("RightHipRoll"));
    damping(cg.GetJointIdv("RightHipYaw")) = d_hipyaw * cg.GetQvelSX()(cg.GetJointIdv("RightHipYaw"));
    damping(cg.GetJointIdv("RightHipPitch")) = d_hippitch * cg.GetQvelSX()(cg.GetJointIdv("RightHipPitch"));
    damping(cg.GetJointIdv("RightKneePitch")) = d_knee * cg.GetQvelSX()(cg.GetJointIdv("RightKneePitch"));
    damping(cg.GetJointIdv("RightShinPitch")) = d_shinspring * cg.GetQvelSX()(cg.GetJointIdv("RightShinPitch"));
    damping(cg.GetJointIdv("RightTarsusPitch")) = d_tarsus * cg.GetQvelSX()(cg.GetJointIdv("RightTarsusPitch"));
    damping(cg.GetJointIdv("RightAchillesSpring")) = d_heelspring * cg.GetQvelSX()(cg.GetJointIdv("RightAchillesSpring"));
    damping(cg.GetJointIdv("RightFootPitch")) = d_foot * cg.GetQvelSX()(cg.GetJointIdv("RightFootPitch"));

    // Add damping forces to bias vector
    h += damping;

    // Generate code for bias vector
    casadi::SX h_sx;
    pinocchio::casadi::copy(h, h_sx);
    cg.GenerateCode("bias_vector", {cg.GetQposSX(), cg.GetQvelSX()}, {h_sx});

    // Actuation matrix (filled with gear ratios)
    casadi::SX B = casadi::SX::zeros(model.nv, 10);
    B(cg.GetJointIdv("LeftHipRoll"), 0) = model.rotorGearRatio[cg.GetJointIdv("LeftHipRoll")];
    B(cg.GetJointIdv("LeftHipYaw"), 1) = model.rotorGearRatio[cg.GetJointIdv("LeftHipYaw")];
    B(cg.GetJointIdv("LeftHipPitch"), 2) = model.rotorGearRatio[cg.GetJointIdv("LeftHipPitch")];
    B(cg.GetJointIdv("LeftKneePitch"), 3) = model.rotorGearRatio[cg.GetJointIdv("LeftKneePitch")];
    B(cg.GetJointIdv("LeftFootPitch"), 4) = model.rotorGearRatio[cg.GetJointIdv("LeftFootPitch")];

    B(cg.GetJointIdv("RightHipRoll"), 5) = model.rotorGearRatio[cg.GetJointIdv("RightHipRoll")];
    B(cg.GetJointIdv("RightHipYaw"), 6) = model.rotorGearRatio[cg.GetJointIdv("RightHipYaw")];
    B(cg.GetJointIdv("RightHipPitch"), 7) = model.rotorGearRatio[cg.GetJointIdv("RightHipPitch")];
    B(cg.GetJointIdv("RightKneePitch"), 8) = model.rotorGearRatio[cg.GetJointIdv("RightKneePitch")];
    B(cg.GetJointIdv("RightFootPitch"), 9) = model.rotorGearRatio[cg.GetJointIdv("RightFootPitch")];

    cg.GenerateCode("actuation_map", {cg.GetQposSX()}, {densify(B)});

    // Add frames for the "ankle" (toe joint, see URDF)
    cg.GenerateEndEffectorData("RightFootPitch");
    cg.GenerateEndEffectorData("LeftFootPitch");

    return 0;
}