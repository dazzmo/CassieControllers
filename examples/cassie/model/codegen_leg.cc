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
        {.name = "heel_tip", .parent_joint = "LeftAchillesSpring", .parent_frame = "leftheelspring", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "achilles_socket", .parent_joint = "LeftHipPitch", .parent_frame = "lefthippitch", .r = Eigen::Vector3d(0.0, 0.0, 0.045), .R = Eigen::Matrix3d::Identity()},
        // Foot contact locations
        {.name = "foot", .parent_joint = "LeftFootPitch", .parent_frame = "leftfoot", .r = Eigen::Vector3d(0.11877, -0.01, 0.0), .R = foot_rot_q.matrix()}};

    // Add frames to model
    for (BodySite& site : sites) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint), model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

    std::vector<BodySite> contact_locations{
        {.name = "foot_front", .parent_joint = "LeftFootPitch", .parent_frame = "foot", .r = Eigen::Vector3d(0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()},
        {.name = "foot_back", .parent_joint = "LeftFootPitch", .parent_frame = "foot", .r = Eigen::Vector3d(-0.09, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()}};

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

    // If using stiff model
    casadi::SX cl_stiff = casadi::SX::zeros(3);
    cl_stiff(0) = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    cl_stiff(1) = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    // Assumption that
    cl_stiff(2) = cs_q(model.joints[model.getJointId("LeftKneePitch")].idx_q()) -
                  cs_q(model.joints[model.getJointId("LeftTarsusPitch")].idx_q()) +
                  13 * M_PI / 180.0;

    // If using spring model
    const double length = 0.5012;
    ADData::Vector3 dl = ad_data.oMf[ad_model.getFrameId("achilles_socket")].translation() - ad_data.oMf[ad_model.getFrameId("heel_tip")].translation();

    // Create constraints
    casadi::SX cl = dl.squaredNorm() - length * length;
    // Get jacobian of constraint and time derivative
    casadi::SX Jcl = casadi::SX::jacobian(cl, cs_q);
    casadi::SX dJcldt = casadi::SX::jacobian(casadi::SX::mtimes(Jcl, cs_v), cs_q);

    // Estimation for heel spring deflection
    casadi::SX q_hs = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());
    casadi::SX g_cl_hs = casadi::SX::gradient(cl, q_hs);

    // Compute mass matrix and bias forces
    pinocchio::crba(ad_model, ad_data, q_ad);
    pinocchio::rnea(ad_model, ad_data, q_ad, v_ad, zero_ad);
    pinocchio::inverse(ad_data.M, ad_data.Minv);

    // Create null-space projector for dynamics
    casadi::SX Minv;
    casadi::SX M, h, B;
    pinocchio::casadi::copy(ad_data.M, M);
    pinocchio::casadi::copy(ad_data.tau, h);
    pinocchio::casadi::copy(ad_data.Minv, Minv);
    // Convert from upper triangular matrix to symmetric
    for (int i = 0; i < model.nv; i++) {
        for (int j = i; j < model.nv; j++) {
            M(j, i) = M(i, j);
            Minv(j, i) = Minv(i, j);
        }
    }
    casadi::SX JMJT = mtimes(Jcl, mtimes(Minv, transpose(Jcl)));

    // Spring deflection
    casadi::SX spring_forces = casadi::SX::zeros(model.nv);
    double k_knee_stiffness = 2300.0;
    double k_heel_stiffness = 2400.0;
    double b_knee_damping = 4.0;
    double b_heel_damping = 4.0;

    ADScalar ql_knee = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    ADScalar ql_heel = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());

    ADScalar vl_knee = cs_v(model.joints[model.getJointId("LeftShinPitch")].idx_v());
    ADScalar vl_heel = cs_v(model.joints[model.getJointId("LeftAchillesSpring")].idx_v());

    spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) = k_knee_stiffness * (ql_knee) + b_knee_damping * (vl_knee);
    spring_forces(model.joints[model.getJointId("LeftAchillesSpring")].idx_v()) = k_heel_stiffness * (ql_heel) + b_heel_damping * (vl_heel);

    // Create actuation matrix
    B = casadi::SX::zeros(model.nv, 5);
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 0) = 25.0;
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 1) = 25.0;
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 2) = 16.0;
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = 16.0;
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = 50.0;

    // Leg inverse kinematics (assume rigid connection for IK)

    // Function to produce kinematics and jacobian
    std::vector<casadi::Function> functions{
        casadi::Function(model.name + "_mass_matrix", casadi::SXVector{cs_q}, casadi::SXVector{densify(M)}),
        casadi::Function(model.name + "_mass_matrix_inv", casadi::SXVector{cs_q}, casadi::SXVector{densify(Minv)}),
        casadi::Function(model.name + "_bias_vector", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(h)}),
        casadi::Function(model.name + "_spring_forces", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(spring_forces)}),
        casadi::Function(model.name + "_heel_spring_constraint_inertia", casadi::SXVector{cs_q}, casadi::SXVector{densify(JMJT)}),
        casadi::Function(model.name + "_heel_spring_constraint", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{cl, densify(Jcl), densify(dJcldt)}),
        casadi::Function(model.name + "_actuation", casadi::SXVector{cs_q}, casadi::SXVector{densify(B)})};

    // Also compute end-effector positions and jacobians
    ADData::Matrix6x J(6, model.nv), dJdt(6, model.nv);
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad);
    pinocchio::computeJointJacobians(ad_model, ad_data, q_ad);
    pinocchio::computeJointJacobiansTimeVariation(ad_model, ad_data, q_ad, v_ad);
    pinocchio::updateFramePlacements(ad_model, ad_data);

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

        functions.push_back(casadi::Function(model.name + "_" + site.name,
                                             casadi::SXVector{cs_q, cs_v},
                                             casadi::SXVector{cs_p, cs_J, cs_dJdq}));
    }

    casadi::Dict opts;
    opts["with_header"] = true;
    for (casadi::Function& fun : functions) {
        casadi::CodeGenerator cg(fun.name(), opts);
        cg.add(fun);
        cg.generate(argv[2]);
    }

    return 0;
}
