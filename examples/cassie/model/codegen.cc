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
    if (argc != 4) {
        // std::cerr <<
    }

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
    pinocchio::urdf::buildModel(argv[1], model, true);

    ConfigVector q(model.nq);
    TangentVector v(model.nv);

    // Foot frame rotation matrix
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-M_PI_2, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond foot_rot_q = yawAngle * pitchAngle * rollAngle;

    std::vector<BodySite> sites = {
        {.name = "heel_l_tip", .parent_joint = "LeftAchillesSpring", .parent_frame = "leftheelspring", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "heel_r_tip", .parent_joint = "RightAchillesSpring", .parent_frame = "rightheelspring", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "achilles_l_socket", .parent_joint = "LeftHipPitch", .parent_frame = "lefthippitch", .r = Eigen::Vector3d(0.0, 0.0, 0.045), .R = Eigen::Matrix3d::Identity()},
        {.name = "achilles_r_socket", .parent_joint = "RightHipPitch", .parent_frame = "righthippitch", .r = Eigen::Vector3d(0.0, 0.0, 0.045), .R = Eigen::Matrix3d::Identity()},
        // Foot contact locations
        {.name = "foot_l", .parent_joint = "LeftFootPitch", .parent_frame = "leftfoot", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = foot_rot_q.matrix()},
        {.name = "foot_r", .parent_joint = "RightFootPitch", .parent_frame = "rightfoot", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = foot_rot_q.matrix()}};

    // Add frames to model
    for (BodySite& site : sites) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint), model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

    std::vector<BodySite> contact_locations{
        {.name = "foot_l_front", .parent_joint = "LeftFootPitch", .parent_frame = "foot_l", .r = Eigen::Vector3d(0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "foot_l_back", .parent_joint = "LeftFootPitch", .parent_frame = "foot_l", .r = Eigen::Vector3d(-0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "foot_r_front", .parent_joint = "RightFootPitch", .parent_frame = "foot_r", .r = Eigen::Vector3d(0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "foot_r_back", .parent_joint = "RightFootPitch", .parent_frame = "foot_r", .r = Eigen::Vector3d(-0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()}};

    for (BodySite& site : contact_locations) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint), model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

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

    TangentVectorAD zero_ad = TangentVectorAD::Zero(model.nv);

    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad, zero_ad);
    pinocchio::updateFramePlacements(ad_model, ad_data);

    // Achilles constraints
    const double length = 0.5012;
    ADData::Vector3 dl = ad_data.oMf[ad_model.getFrameId("achilles_l_socket")].translation() - ad_data.oMf[ad_model.getFrameId("heel_l_tip")].translation();
    ADData::Vector3 dr = ad_data.oMf[ad_model.getFrameId("achilles_r_socket")].translation() - ad_data.oMf[ad_model.getFrameId("heel_r_tip")].translation();

    // Create constraints
    Eigen::Vector<casadi::SX, 2> cl;
    cl << dl.squaredNorm() - length * length, dr.squaredNorm() - length * length;

    casadi::SX achilles_con;
    pinocchio::casadi::copy(cl, achilles_con);

    // Get joints of robot (not floating base)
    casadi::SX cs_qj = cs_q(casadi::Slice(7, model.nq));
    casadi::SX cs_vj = cs_v(casadi::Slice(6, model.nv));

    // Get jacobian of constraint and time derivative
    casadi::SX J_achilles = casadi::SX::jacobian(achilles_con, cs_qj);
    casadi::SX dJdt_achilles = casadi::SX::jacobian(casadi::SX::mtimes(J_achilles, cs_vj), cs_qj);
    // Create full body jacobian
    J_achilles = casadi::SX::horzcat({casadi::SX::zeros(2, 6), J_achilles});
    dJdt_achilles = casadi::SX::horzcat({casadi::SX::zeros(2, 6), dJdt_achilles});

    // Compute contact jacobians
    ADData::Matrix6x J(6, model.nv), dJdt(6, model.nv);
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad);
    pinocchio::computeJointJacobians(ad_model, ad_data, q_ad);
    pinocchio::computeJointJacobiansTimeVariation(ad_model, ad_data, q_ad, v_ad);
    pinocchio::updateFramePlacements(ad_model, ad_data);

    casadi::Dict opts;
    opts["with_header"] = true;

    for (BodySite& site : contact_locations) {
        J.setZero();
        dJdt.setZero();
        pinocchio::getFrameJacobian(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED, J);
        pinocchio::getFrameJacobianTimeVariation(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED, dJdt);
        ADData::Vector3 dJdq = dJdt.topRows(3) * v_ad;

        // Create function
        casadi::SX cs_p, cs_J, cs_dJdq;
        pinocchio::casadi::copy(ad_data.oMf[ad_model.getFrameId(site.name)].translation(), cs_p);
        pinocchio::casadi::copy(J.topRows(3), cs_J);
        pinocchio::casadi::copy(dJdt.topRows(3), cs_dJdq);
        cs_J = casadi::SX::densify(cs_J);
        cs_dJdq = casadi::SX::densify(cs_dJdq);

        casadi::Function fp(model.name + "_" + site.name,
                            casadi::SXVector{cs_q, cs_v},
                            casadi::SXVector{cs_p, cs_J, cs_dJdq});

        // Generate code
        casadi::CodeGenerator cg_p(model.name + "_" + site.name, opts);
        cg_p.add(fp);
        cg_p.generate(argv[2]);
    }

    pinocchio::crba(ad_model, ad_data, q_ad);
    pinocchio::rnea(ad_model, ad_data, q_ad, v_ad, zero_ad);

    pinocchio::centerOfMass(ad_model, ad_data);
    pinocchio::jacobianCenterOfMass(ad_model, ad_data, q_ad);

    // Create null-space projector for dynamics
    casadi::SX& cs_J = J_achilles;
    casadi::SX cs_JT = transpose(cs_J);
    casadi::SX cs_Minv;
    pinocchio::casadi::copy(ad_data.Minv, cs_Minv);
    casadi::SX JMJT = mtimes(cs_J, mtimes(cs_Minv, transpose(cs_J)));
    casadi::SX N = casadi::SX::eye(model.nv) - mtimes(transpose(cs_J),
                                                      mtimes(casadi::SX::pinv(JMJT),
                                                             mtimes(cs_J, cs_Minv)));

    casadi::SX gamma = -mtimes(transpose(cs_J), mtimes(casadi::SX::pinv(JMJT), mtimes(dJdt_achilles, cs_v)));

    casadi::SX M, h, B, x_com, J_com, dJdq_com;
    pinocchio::casadi::copy(ad_data.M, M);
    pinocchio::casadi::copy(ad_data.tau, h);
    pinocchio::casadi::copy(ad_data.com[0], x_com);
    pinocchio::casadi::copy(ad_data.Jcom, J_com);
    pinocchio::casadi::copy(ad_data.acom[0], dJdq_com);

    // Spring deflection
    casadi::SX spring_forces = casadi::SX::zeros(model.nv);
    double k_knee_stiffness = 4.0;
    double k_heel_stiffness = 2000.0;
    double b_knee_damping = 1.0;
    double b_heel_damping = 1.0;

    ADScalar ql_knee = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    ADScalar qr_knee = cs_q(model.joints[model.getJointId("RightShinPitch")].idx_q());
    ADScalar ql_heel = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    ADScalar qr_heel = cs_q(model.joints[model.getJointId("RightAchillesSpring")].idx_q());

    ADScalar vl_knee = cs_v(model.joints[model.getJointId("LeftShinPitch")].idx_v());
    ADScalar vr_knee = cs_v(model.joints[model.getJointId("RightShinPitch")].idx_v());
    ADScalar vl_heel = cs_v(model.joints[model.getJointId("LeftAchillesSpring")].idx_v());
    ADScalar vr_heel = cs_v(model.joints[model.getJointId("RightAchillesSpring")].idx_v());

    spring_forces(model.joints[model.getJointId("RightShinPitch")].idx_v()) = k_knee_stiffness * (qr_knee) + b_knee_damping * (vr_knee);
    spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) = k_knee_stiffness * (ql_knee) + b_knee_damping * (vl_knee);
    spring_forces(model.joints[model.getJointId("RightAchillesSpring")].idx_v()) = k_heel_stiffness * (qr_heel) + b_heel_damping * (vr_heel);
    spring_forces(model.joints[model.getJointId("LeftAchillesSpring")].idx_v()) = k_heel_stiffness * (ql_heel) + b_heel_damping * (vl_heel);

    // Create actuation matrix
    B = casadi::SX::zeros(model.nv, 10);
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 0) = 25.0;
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 1) = 25.0;
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 2) = 16.0;
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = 16.0;
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = 50.0;

    B(model.joints[model.getJointId("RightHipRoll")].idx_v(), 5) = 25.0;
    B(model.joints[model.getJointId("RightHipYaw")].idx_v(), 6) = 25.0;
    B(model.joints[model.getJointId("RightHipPitch")].idx_v(), 7) = 16.0;
    B(model.joints[model.getJointId("RightKneePitch")].idx_v(), 8) = 16.0;
    B(model.joints[model.getJointId("RightFootPitch")].idx_v(), 9) = 50.0;

    std::vector<casadi::Function> functions{
        casadi::Function(model.name + "_mass_matrix", casadi::SXVector{cs_q}, casadi::SXVector{densify(M)}),
        casadi::Function(model.name + "_bias_vector", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(h)}),
        casadi::Function(model.name + "_nullspace_projector", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(N), densify(gamma)}),
        casadi::Function(model.name + "_spring_forces", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(spring_forces)}),
        casadi::Function(model.name + "_actuation_matrix", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(B)}),
        casadi::Function(model.name + "_centre_of_mass", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{x_com, densify(J_com), dJdq_com})};

    for (casadi::Function& fun : functions) {
        casadi::CodeGenerator cg(fun.name(), opts);
        cg.add(fun);
        cg.generate(argv[2]);
    }

    return 0;
}
