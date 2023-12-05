#include "code_generator.h"

int main(int argc, char* argv[]) {

    // Initialise model from Cassie urdf
    CodeGenerator cg("./cassie_fixed.urdf");
    cg.SetCodeGenerationDestination(argv[1]);

    // Define rotation matrix for sole of foot
    // TODO: Double-check the order (ZYX vs. XYZ)
    // Sole of foot frame (https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model)
    Eigen::AngleAxisd rollAngle(-M_PI_2, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond foot_rot_q = yawAngle * pitchAngle * rollAngle;
    
    // Create useful reference frames
    cg.AddReferenceFrame("left_foot", "LeftFootPitch", "leftfoot",
                         Eigen::Vector3d(0.01762, 0.05219, 0.0), foot_rot_q.matrix());
    cg.AddReferenceFrame("right_foot", "RightFootPitch", "rightfoot",
                         Eigen::Vector3d(0.01762, 0.05219, 0.0), foot_rot_q.matrix());

    // https://github.com/agilityrobotics/cassie-doc/wiki/Heel-Spring-Model
    cg.AddReferenceFrame("left_heel_tip", "LeftAchillesSpring", "leftheelspring",
                         Eigen::Vector3d(0.11877, -0.01, 0.0), Eigen::Matrix3d::Identity());
    cg.AddReferenceFrame("right_heel_tip", "RightAchillesSpring", "rightheelspring",
                         Eigen::Vector3d(0.11877, -0.01, 0.0), Eigen::Matrix3d::Identity());

    // https://github.com/agilityrobotics/cassie-doc/wiki/Thigh-Model
    cg.AddReferenceFrame("left_achilles_socket", "LeftHipPitch", "lefthippitch",
                         Eigen::Vector3d(0.0, 0.0, 0.045), Eigen::Matrix3d::Identity());
    cg.AddReferenceFrame("right_achilles_socket", "RightHipPitch", "righthippitch",
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

    // Achilles rod on Cassie constrains each end to be 501.2mm apart:
    // https://github.com/agilityrobotics/cassie-doc/wiki/Achilles-Rod-Model
    const double achilles_length = 0.5012;

    // Create constraints, including for leaf springs, which we set to 0 deflection
    CodeGenerator::ADData::Vector3 dl = data.oMf[model.getFrameId("left_achilles_socket")].translation() -
                                        data.oMf[model.getFrameId("left_heel_tip")].translation();
    CodeGenerator::ADData::Vector3 dr = data.oMf[model.getFrameId("right_achilles_socket")].translation() -
                                        data.oMf[model.getFrameId("right_heel_tip")].translation();

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
    // TODO: Check if I have to split it up into left and right constraints!
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

    // TODO: Up to spring dynamics. Continue later...

    return 0;
}