#include "code_generator.h"

// TODO: Read all the constants from a .yaml file or something

int main(int argc, char* argv[]) {
    CodeGenerator cg("./cassie_leg.urdf");
    cg.SetCodeGenerationDestination(argv[1]);

    // Create reference frames
    // Sole of foot frame (https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model)
    // TODO: Double-check the order (ZYX vs. XYZ)
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
    model.rotorGearRatio[model.joints[model.getJointId("LeftHipRoll")].idx_v()] = 25.0;
    model.rotorGearRatio[model.joints[model.getJointId("LeftHipYaw")].idx_v()] = 25.0;
    model.rotorGearRatio[model.joints[model.getJointId("LeftHipPitch")].idx_v()] = 16.0;
    model.rotorGearRatio[model.joints[model.getJointId("LeftKneePitch")].idx_v()] = 16.0;
    model.rotorGearRatio[model.joints[model.getJointId("LeftFootPitch")].idx_v()] = 50.0;

    model.rotorInertia[model.joints[model.getJointId("LeftHipRoll")].idx_v()] = 6.10e-05;
    model.rotorInertia[model.joints[model.getJointId("LeftHipYaw")].idx_v()] = 6.10e-05;
    model.rotorInertia[model.joints[model.getJointId("LeftHipPitch")].idx_v()] = 3.65e-04;
    model.rotorInertia[model.joints[model.getJointId("LeftKneePitch")].idx_v()] = 3.65e-04;
    model.rotorInertia[model.joints[model.getJointId("LeftFootPitch")].idx_v()] = 4.90e-06;

    // Achilles rod on Cassie constrains each end to be 501.2mm apart:
    // https://github.com/agilityrobotics/cassie-doc/wiki/Achilles-Rod-Model
    const double achilles_length = 0.5012;

    // Create constraint
    CodeGenerator::ADData::Vector3 dl = data.oMf[model.getFrameId("achilles_socket")].translation() -
                                        data.oMf[model.getFrameId("heel_tip")].translation();
    casadi::SX cl = dl.squaredNorm() - achilles_length * achilles_length;

    // Get Jacobian of constraint, its time-derivative, and the Hessian
    casadi::SX Jcl = jacobian(cl, cg.GetConfigurationVectorSX());
    casadi::SX Hcl = hessian(cl, cg.GetConfigurationVectorSX());
    // dJdt = dJdq * dqdt by chain rule
    casadi::SX dJcldt = jacobian(mtimes(Jcl, cg.GetTangentVectorSX()), cg.GetConfigurationVectorSX());

    cg.GenerateCode("achilles_rod_constraint", {cg.GetConfigurationVectorSX(), cg.GetTangentVectorSX()},
                    {densify(cl), densify(Jcl), densify(mtimes(dJcldt, cg.GetTangentVectorSX())), densify(Hcl)});

    // Bias vector
    CodeGenerator::TangentVectorAD h = pinocchio::rnea<CodeGenerator::ADScalar>(model, data,
                                                                                cg.GetConfigurationVector(),
                                                                                cg.GetTangentVector(),
                                                                                CodeGenerator::TangentVectorAD::Zero(model.nv));

    // Spring dynamics parameters from:
    // https://github.com/jpreher/cassie_description/blob/master/MATLAB/Cassie_v4.m#L193
    CodeGenerator::TangentVectorAD spring_forces = CodeGenerator::TangentVectorAD::Zero(model.nv);
    double k_knee_stiffness = 2300.0;
    double k_heel_stiffness = 2000.0;
    double b_knee_damping = 4.6;
    double b_heel_damping = 4.0;

    // Extract joint coordinates of springs
    casadi::SX q_knee = cg.GetConfigurationVectorSX()(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    casadi::SX q_heel = cg.GetConfigurationVectorSX()(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());

    casadi::SX v_knee = cg.GetTangentVectorSX()(model.joints[model.getJointId("LeftShinPitch")].idx_v());
    casadi::SX v_heel = cg.GetTangentVectorSX()(model.joints[model.getJointId("LeftAchillesSpring")].idx_v());

    // Add to spring forces
    spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) = k_knee_stiffness * (q_knee) + b_knee_damping * (v_knee);
    spring_forces(model.joints[model.getJointId("LeftAchillesSpring")].idx_v()) = k_heel_stiffness * (q_heel) + b_heel_damping * (v_heel);

    // Add spring forces to bias vector
    h += spring_forces;

    // Damping forces
    CodeGenerator::TangentVectorAD damping(model.nv);
    damping.setConstant(1.0);  // TODO: Provide better estimate of these terms
    h += damping;

    casadi::SX h_sx;
    pinocchio::casadi::copy(h, h_sx);
    cg.GenerateCode("bias_vector", {cg.GetConfigurationVectorSX(), cg.GetTangentVectorSX()}, {h_sx});

    // Actuation matrix
    casadi::SX B = casadi::SX::zeros(model.nv, 5);
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 0) = model.rotorGearRatio[model.joints[model.getJointId("LeftHipRoll")].idx_v()];
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 1) = model.rotorGearRatio[model.joints[model.getJointId("LeftHipYaw")].idx_v()];
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 2) = model.rotorGearRatio[model.joints[model.getJointId("LeftHipPitch")].idx_v()];
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = model.rotorGearRatio[model.joints[model.getJointId("LeftKneePitch")].idx_v()];
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = model.rotorGearRatio[model.joints[model.getJointId("LeftFootPitch")].idx_v()];

    cg.GenerateCode("actuation_map", {cg.GetConfigurationVectorSX()}, {B});

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
