// #include "code_generator.h"

// // TODO: Read all the constants from a .yaml file
// // TODO: Provide better estimate of damping terms? (see other Cassie groups)

// int main(int argc, char* argv[]) {

//     // Initialise model from Cassie urdf (just one leg)
//     CodeGenerator cg("./cassie_leg.urdf");
//     cg.SetCodeGenerationDestination(argv[1]);

//     /********** Create reference frames **********/

//     // TODO: Double-check the order (ZYX vs. XYZ)
//     // Sole of foot frame (https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model)
//     Eigen::AngleAxisd rollAngle(-M_PI_2, Eigen::Vector3d::UnitX());
//     Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
//     Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
//     Eigen::Quaterniond foot_rot_q = yawAngle * pitchAngle * rollAngle;

//     cg.AddReferenceFrame(
//         "foot",
//         "LeftFootPitch",
//         "leftfoot",
//         Eigen::Vector3d(0.01762, 0.05219, 0.0),
//         foot_rot_q.matrix());

//     // https://github.com/agilityrobotics/cassie-doc/wiki/Heel-Spring-Model
//     cg.AddReferenceFrame(
//         "heel_tip",
//         "LeftAchillesSpring",
//         "leftheelspring",
//         Eigen::Vector3d(0.11877, -0.01, 0.0),
//         Eigen::Matrix3d::Identity());

//     // https://github.com/agilityrobotics/cassie-doc/wiki/Thigh-Model
//     cg.AddReferenceFrame(
//         "achilles_socket",
//         "LeftHipPitch",
//         "lefthippitch",
//         Eigen::Vector3d(0.0, 0.0, 0.045),
//         Eigen::Matrix3d::Identity());

//     // Get model and data references
//     CodeGenerator::ADModel& model = cg.GetModel();
//     CodeGenerator::ADData& data = cg.GetData();

//     // Compute inertia matrix
//     cg.GenerateInertiaMatrix();

//     // Add gearing and rotor inertia information (all from Cassie wiki)
//     model.rotorGearRatio[cg.GetJointIdv("LeftHipRoll")] = 25.0;
//     model.rotorGearRatio[cg.GetJointIdv("LeftHipYaw")] = 25.0;
//     model.rotorGearRatio[cg.GetJointIdv("LeftHipPitch")] = 16.0;
//     model.rotorGearRatio[cg.GetJointIdv("LeftKneePitch")] = 16.0;
//     model.rotorGearRatio[cg.GetJointIdv("LeftFootPitch")] = 50.0;

//     model.rotorInertia[cg.GetJointIdv("LeftHipRoll")] = 6.10e-05;
//     model.rotorInertia[cg.GetJointIdv("LeftHipYaw")] = 6.10e-05;
//     model.rotorInertia[cg.GetJointIdv("LeftHipPitch")] = 3.65e-04;
//     model.rotorInertia[cg.GetJointIdv("LeftKneePitch")] = 3.65e-04;
//     model.rotorInertia[cg.GetJointIdv("LeftFootPitch")] = 4.90e-06;

//     // Achilles rod on Cassie constrains each end to be 501.2mm apart:
//     // https://github.com/agilityrobotics/cassie-doc/wiki/Achilles-Rod-Model
//     const double achilles_length = 0.5012;

//     // Create constraint
//     CodeGenerator::ADData::Vector3 dl = data.oMf[model.getFrameId("achilles_socket")].translation() -
//                                         data.oMf[model.getFrameId("heel_tip")].translation();
//     casadi::SX cl = dl.squaredNorm() - achilles_length * achilles_length;

//     // // Uncomment for a stiff model TODO: Not finished/working yet
//     // casadi::SX cl = casadi::SX::zeros(3);
//     // cl(0) = cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring"));
//     // cl(1) = cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch"));
//     // cl(2) = cg.GetQposSX()(cg.GetJointIdq("LeftKneePitch")) -
//     //         cg.GetQposSX()(cg.GetJointIdq("LeftTarsusPitch")) + 13 * M_PI / 180.0;

//     // Get Jacobian of the closed-loop constraint, its time-derivative, and the Hessian
//     // Note that dJdt = dJdq * dqdt by chain rule
//     casadi::SX Jcl = jacobian(cl, cg.GetQposSX());
//     casadi::SX Hcl = hessian(cl, cg.GetQposSX());
//     casadi::SX dJcldt = jacobian(mtimes(Jcl, cg.GetQvelSX()), cg.GetQposSX());

//     // Generate code for constraint
//     cg.GenerateCode(
//         "achilles_rod_constraint", 
//         {cg.GetQposSX(), cg.GetQvelSX()},
//         {densify(cl), densify(Jcl), densify(mtimes(dJcldt, cg.GetQvelSX())), densify(Hcl)}
//     );

//     // Bias vector (includes gravity, damping, spring forces, etc.)
//     CodeGenerator::TangentVectorAD h = pinocchio::rnea<CodeGenerator::ADScalar>(
//         model, 
//         data,
//         cg.GetQpos(),
//         cg.GetQvel(),
//         CodeGenerator::TangentVectorAD::Zero(model.nv)
//     );

//     // Spring dynamics parameters. Commented values from:
//     // https://github.com/jpreher/cassie_description/blob/master/MATLAB/Cassie_v4.m#L193
//     // Currently-used values from the cassie.xml MuJoCo model
//     // Values will depend on which springs are currently attached to Cassie
//     double k_knee_stiffness = 1500; //2300.0;
//     double k_heel_stiffness = 1250; //2000.0;

//     // Extract joint coordinates of springs
//     casadi::SX q_knee = cg.GetQposSX()(cg.GetJointIdq("LeftShinPitch"));
//     casadi::SX q_heel = cg.GetQposSX()(cg.GetJointIdq("LeftAchillesSpring"));

//     // Add to spring forces
//     CodeGenerator::TangentVectorAD spring_forces = CodeGenerator::TangentVectorAD::Zero(model.nv);
//     spring_forces(cg.GetJointIdv("LeftShinPitch")) = k_knee_stiffness * (q_knee);
//     spring_forces(cg.GetJointIdv("LeftAchillesSpring")) = k_heel_stiffness * (q_heel);

//     // Add spring forces to bias vector
//     h += spring_forces;

//     // Damping forces (from MuJoCo model, cassie.xml)
//     // Commented values from same source as springs
//     double d_lhiproll     = 1.0;
//     double d_lhipyaw      = 1.0;
//     double d_lhippitch    = 1.0;
//     double d_lknee        = 1.0;
//     double d_lshinspring  = 0.1;     //4.6
//     double d_ltarsus      = 0.1;
//     double d_lheelspring  = 0.001;   //4.0
//     double d_lfoot        = 1.0;

//     // Add to damping forces
//     // TODO: Could do this more nicely with a diagonal matrix
//     CodeGenerator::TangentVectorAD damping(model.nv);
//     damping(cg.GetJointIdv("LeftHipRoll"))        = d_lhiproll    * cg.GetQvelSX()(cg.GetJointIdv("LeftHipRoll"));
//     damping(cg.GetJointIdv("LeftHipYaw"))         = d_lhipyaw     * cg.GetQvelSX()(cg.GetJointIdv("LeftHipYaw"));
//     damping(cg.GetJointIdv("LeftHipPitch"))       = d_lhippitch   * cg.GetQvelSX()(cg.GetJointIdv("LeftHipPitch"));
//     damping(cg.GetJointIdv("LeftKneePitch"))      = d_lknee       * cg.GetQvelSX()(cg.GetJointIdv("LeftKneePitch"));
//     damping(cg.GetJointIdv("LeftShinPitch"))      = d_lshinspring * cg.GetQvelSX()(cg.GetJointIdv("LeftShinPitch"));
//     damping(cg.GetJointIdv("LeftTarsusPitch"))    = d_ltarsus     * cg.GetQvelSX()(cg.GetJointIdv("LeftTarsusPitch"));
//     damping(cg.GetJointIdv("LeftAchillesSpring")) = d_lheelspring * cg.GetQvelSX()(cg.GetJointIdv("LeftAchillesSpring"));
//     damping(cg.GetJointIdv("LeftFootPitch"))      = d_lfoot       * cg.GetQvelSX()(cg.GetJointIdv("LeftFootPitch"));

//     // Add damping forces to bias vector
//     h += damping;

//     // Generate code for bias vector
//     casadi::SX h_sx;
//     pinocchio::casadi::copy(h, h_sx);
//     cg.GenerateCode("bias_vector", {cg.GetQposSX(), cg.GetQvelSX()}, {h_sx});

//     // Actuation matrix (filled with gear ratios)
//     casadi::SX B = casadi::SX::zeros(model.nv, 5);
//     B(cg.GetJointIdv("LeftHipRoll"), 0)   = model.rotorGearRatio[cg.GetJointIdv("LeftHipRoll")];
//     B(cg.GetJointIdv("LeftHipYaw"), 1)    = model.rotorGearRatio[cg.GetJointIdv("LeftHipYaw")];
//     B(cg.GetJointIdv("LeftHipPitch"), 2)  = model.rotorGearRatio[cg.GetJointIdv("LeftHipPitch")];
//     B(cg.GetJointIdv("LeftKneePitch"), 3) = model.rotorGearRatio[cg.GetJointIdv("LeftKneePitch")];
//     B(cg.GetJointIdv("LeftFootPitch"), 4) = model.rotorGearRatio[cg.GetJointIdv("LeftFootPitch")];

//     cg.GenerateCode("actuation_map", {cg.GetQposSX()}, {densify(B)});

//     // Generate end-effector data
//     // https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model
//     cg.GenerateEndEffectorData("foot_front",
//                                "LeftFootPitch", "foot",
//                                Eigen::Vector3d(0.09, 0.0, 0.0),
//                                Eigen::Matrix3d::Identity());

//     cg.GenerateEndEffectorData("foot_back",
//                                "LeftFootPitch", "foot",
//                                Eigen::Vector3d(-0.09, 0.0, 0.0),
//                                Eigen::Matrix3d::Identity());

//     cg.GenerateEndEffectorData("ankle",
//                                "LeftFootPitch", "leftfoot",
//                                Eigen::Vector3d(0.0, 0.0, 0.0),
//                                Eigen::Matrix3d::Identity());

//     return 0;
// }

#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/center-of-mass-derivatives.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/autodiff/casadi.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"

struct BodySite {
    std::string name;
    std::string parent_joint;
    std::string parent_frame;
    Eigen::Vector3d r;
    Eigen::Matrix3d R;
};

int main(int argc, char* argv[]) {
    typedef double Scalar;
    typedef pinocchio::ModelTpl<Scalar> Model;
    typedef Model::Data Data;
    typedef Model::ConfigVectorType ConfigVector;
    typedef Model::TangentVectorType TangentVector;

    typedef casadi::SX ADScalar;
    typedef pinocchio::ModelTpl<ADScalar> ADModel;
    typedef ADModel::Data ADData;
    typedef ADModel::ConfigVectorType ConfigVectorAD;
    typedef ADModel::TangentVectorType TangentVectorAD;

    Model model;
    pinocchio::urdf::buildModel("cassie_leg.urdf", model, true);

    ConfigVector q(model.nq);
    TangentVector v(model.nv);

    // Foot frame rotation matrix
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-M_PI_2, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond foot_rot_q = yawAngle * pitchAngle * rollAngle;

    std::vector<BodySite> sites = {
        {.name = "heel_tip", .parent_joint = "LeftAchillesSpring", .parent_frame = "leftheelspring", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "achilles_socket", .parent_joint = "LeftHipPitch", .parent_frame = "lefthippitch", .r = Eigen::Vector3d(0.0, 0.0, 0.045), .R = Eigen::Matrix3d::Identity()},
        // Foot contact locations
        {.name = "foot", .parent_joint = "LeftFootPitch", .parent_frame = "leftfoot", .r = Eigen::Vector3d(0.01762, 0.05219, 0.0), .R = foot_rot_q.matrix()}};

    // Add frames to model
    for (BodySite& site : sites) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint),
                                        model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

    std::vector<BodySite> contact_locations{
        {.name = "foot_front", .parent_joint = "LeftFootPitch", .parent_frame = "foot", .r = Eigen::Vector3d(0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "foot_back", .parent_joint = "LeftFootPitch", .parent_frame = "foot", .r = Eigen::Vector3d(-0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "ankle", .parent_joint = "LeftFootPitch", .parent_frame = "leftfoot", .r = Eigen::Vector3d(0.0, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()}};

    for (BodySite& site : contact_locations) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint),
                                        model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

    // Add gearing and rotor inertia information
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

    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    casadi::SX cs_q = casadi::SX::sym("q", model.nq);
    ConfigVectorAD q_ad(model.nq);
    for (Eigen::DenseIndex k = 0; k < model.nq; ++k) {
        q_ad[k] = cs_q(k);
    }

    casadi::SX cs_v = casadi::SX::sym("v", model.nv);
    TangentVectorAD v_ad(model.nv);
    for (Eigen::DenseIndex k = 0; k < model.nv; ++k) {
        v_ad[k] = cs_v(k);
    }

    pinocchio::framesForwardKinematics(ad_model, ad_data, q_ad);

    // // If using stiff model
    // casadi::SX cl_stiff = casadi::SX::zeros(3);
    // cl_stiff(0) = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    // cl_stiff(1) = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    // cl_stiff(2) = cs_q(model.joints[model.getJointId("LeftKneePitch")].idx_q()) -
    //               cs_q(model.joints[model.getJointId("LeftTarsusPitch")].idx_q()) +
    //               13 * M_PI / 180.0;

    // If using spring model
    const double length = 0.5012;

    // Create constraints
    ADData::Vector3 dl = ad_data.oMf[ad_model.getFrameId("achilles_socket")].translation() -
                         ad_data.oMf[ad_model.getFrameId("heel_tip")].translation();
    casadi::SX cl = dl.squaredNorm() - length * length;

    // Get jacobian of constraint and time derivative
    casadi::SX Jcl = jacobian(cl, cs_q);
    casadi::SX Hcl = hessian(cl, cs_q);
    casadi::SX dJcldt = jacobian(mtimes(Jcl, cs_v), cs_q);

    // // Estimation for heel spring deflection
    // casadi::SX q_hs = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    // casadi::SX g_cl_hs = jacobian(cl, q_hs);

    // Compute mass matrix and bias forces
    Eigen::Matrix<ADScalar, -1, -1> M(model.nv, model.nv);
    TangentVectorAD h(model.nv);
    M = pinocchio::crba(ad_model, ad_data, q_ad);
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
    h = pinocchio::rnea(ad_model, ad_data, q_ad, v_ad, TangentVectorAD::Zero(model.nv));

    // Create null-space projector for dynamics
    casadi::SX cs_M, cs_h;
    pinocchio::casadi::copy(M, cs_M);
    pinocchio::casadi::copy(h, cs_h);

    // Spring deflection
    casadi::SX spring_forces = casadi::SX::zeros(model.nv);
    double k_knee_stiffness = 2300.0;
    double k_heel_stiffness = 2000.0;
    double b_knee_damping = 4.6;
    double b_heel_damping = 4.0;

    ADScalar q_knee = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    ADScalar q_heel = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());

    ADScalar v_knee = cs_v(model.joints[model.getJointId("LeftShinPitch")].idx_v());
    ADScalar v_heel = cs_v(model.joints[model.getJointId("LeftAchillesSpring")].idx_v());

    spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) = k_knee_stiffness * (q_knee) + b_knee_damping * (v_knee);
    spring_forces(model.joints[model.getJointId("LeftAchillesSpring")].idx_v()) = k_heel_stiffness * (q_heel) + b_heel_damping * (v_heel);

    // Create actuation matrix
    casadi::SX B = casadi::SX::zeros(model.nv, 5);
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 0) = 25.0;
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 1) = 25.0;
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 2) = 16.0;
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = 16.0;
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = 50.0;

    // Leg inverse kinematics (assume rigid connection for IK)

    // Function to produce kinematics and jacobian
    std::vector<casadi::Function> functions{
        casadi::Function(model.name + "_mass_matrix", casadi::SXVector{cs_q}, casadi::SXVector{densify(cs_M)}),
        casadi::Function(model.name + "_bias_vector", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(cs_h)}),
        casadi::Function(model.name + "_spring_forces", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(spring_forces)}),
        casadi::Function(model.name + "_achilles_rod_constraint", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(cl), densify(Jcl), densify(mtimes(dJcldt, cs_v)), densify(Hcl)}),
        casadi::Function(model.name + "_actuation_map", casadi::SXVector{cs_q}, casadi::SXVector{densify(B)})};

    // Also compute end-effector positions and jacobians
    ADData::Matrix6x J(6, model.nv), dJdt(6, model.nv);
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad, TangentVectorAD::Zero(model.nv));
    pinocchio::updateFramePlacements(ad_model, ad_data);
    pinocchio::computeJointJacobians(ad_model, ad_data, q_ad);

    for (BodySite& site : contact_locations) {
        J.setZero();
        pinocchio::getFrameJacobian(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED, J);
        pinocchio::MotionTpl<ADScalar> a = pinocchio::getFrameAcceleration(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED);
        // Create function
        casadi::SX cs_p, cs_J, cs_dJdt_v;
        pinocchio::casadi::copy(ad_data.oMf[ad_model.getFrameId(site.name)].translation(), cs_p);
        pinocchio::casadi::copy(J.topRows(3), cs_J);
        pinocchio::casadi::copy(a.linear(), cs_dJdt_v);

        functions.push_back(casadi::Function(model.name + "_" + site.name,
                                             casadi::SXVector{cs_q, cs_v},
                                             casadi::SXVector{densify(cs_p), densify(cs_J), densify(cs_dJdt_v)}));
    }

    casadi::Dict opts;
    opts["with_header"] = true;
    for (casadi::Function& fun : functions) {
        casadi::CodeGenerator cg(fun.name(), opts);
        cg.add(fun);
        cg.generate(argv[1]);
    }

    return 0;
}
