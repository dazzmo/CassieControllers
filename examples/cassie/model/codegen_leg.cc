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

    // If using stiff model
    casadi::SX cl_stiff = casadi::SX::zeros(3);
    cl_stiff(0) = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    cl_stiff(1) = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    cl_stiff(2) = cs_q(model.joints[model.getJointId("LeftKneePitch")].idx_q()) -
                  cs_q(model.joints[model.getJointId("LeftTarsusPitch")].idx_q()) +
                  13 * M_PI / 180.0;

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

    // Estimation for heel spring deflection
    casadi::SX q_hs = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    casadi::SX g_cl_hs = jacobian(cl, q_hs);

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
        casadi::Function(model.name + "_heel_spring_constraint", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(cl), densify(Jcl), densify(mtimes(dJcldt, cs_v)), densify(Hcl)}),
        casadi::Function(model.name + "_actuation", casadi::SXVector{cs_q}, casadi::SXVector{densify(B)})};

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
        casadi::SX cs_p, cs_J, cs_dJdq_v;
        pinocchio::casadi::copy(ad_data.oMf[ad_model.getFrameId(site.name)].translation(), cs_p);
        pinocchio::casadi::copy(J.topRows(3), cs_J);
        pinocchio::casadi::copy(a.linear(), cs_dJdq_v);

        functions.push_back(casadi::Function(model.name + "_" + site.name,
                                             casadi::SXVector{cs_q, cs_v},
                                             casadi::SXVector{densify(cs_p), densify(cs_J), densify(cs_dJdq_v)}));
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
