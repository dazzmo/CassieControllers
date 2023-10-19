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
    pinocchio::urdf::buildModel("arm.urdf", model, true);

    ConfigVector q(model.nq);
    TangentVector v(model.nv);

    std::vector<BodySite> sites = {
        {.name = "tip", .parent_joint = "third_joint", .parent_frame = "third_link", .r = Eigen::Vector3d(0.0, 0.0, 0.0), .R = Eigen::Matrix3d::Identity()}};

    // Add frames to model
    for (BodySite& site : sites) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint),
                                        model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

    std::vector<BodySite> contact_locations{
        {.name = "tip", .parent_joint = "third_joint", .parent_frame = "tip", .r = Eigen::Vector3d(0.0, 0.0, -0.5), .R = Eigen::Matrix3d::Identity()}};

    for (BodySite& site : contact_locations) {
        model.addFrame(pinocchio::Frame(site.name, model.getJointId(site.parent_joint),
                                        model.getFrameId(site.parent_frame),
                                        pinocchio::SE3(site.R, site.r), pinocchio::OP_FRAME));
    }

    // Add gearing and rotor inertia information
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

    // Compute mass matrix and bias forces
    Eigen::Matrix<ADScalar, -1, -1> M(model.nv, model.nv);
    TangentVectorAD h(model.nv);
    M = pinocchio::crba(ad_model, ad_data, q_ad);
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
    h = pinocchio::rnea(ad_model, ad_data, q_ad, v_ad, TangentVectorAD::Zero(model.nv));

    casadi::SX cs_M, cs_h;
    pinocchio::casadi::copy(M, cs_M);
    pinocchio::casadi::copy(h, cs_h);

    // Create actuation matrix
    casadi::SX B = casadi::SX::zeros(model.nv, 3);
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
        casadi::Function(model.name + "_actuation", casadi::SXVector{cs_q}, casadi::SXVector{densify(B)})};

    // Also compute end-effector positions and jacobians
    ADData::Matrix6x J(6, model.nv), dJdt(6, model.nv);
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad, TangentVectorAD::Zero(model.nv));
    pinocchio::updateFramePlacements(ad_model, ad_data);
    pinocchio::computeJointJacobians(ad_model, ad_data, q_ad);

    for (BodySite& site : contact_locations) {
        J.setZero();
        dJdt.setZero();
        pinocchio::getFrameJacobian(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED, J);
        pinocchio::MotionTpl<ADScalar> a = pinocchio::getFrameAcceleration(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED);
        // Create function
        casadi::SX cs_p, cs_J, cs_dJdq;
        pinocchio::casadi::copy(ad_data.oMf[ad_model.getFrameId(site.name)].translation(), cs_p);
        pinocchio::casadi::copy(J.topRows(3), cs_J);
        pinocchio::casadi::copy(a.linear(), cs_dJdq);

        functions.push_back(casadi::Function(model.name + "_" + site.name,
                                             casadi::SXVector{cs_q, cs_v},
                                             casadi::SXVector{densify(cs_p), densify(cs_J), densify(cs_dJdq)}));
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
