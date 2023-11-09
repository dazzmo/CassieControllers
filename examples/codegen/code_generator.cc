#include "code_generator.h"

CodeGenerator::CodeGenerator(const std::string &model_urdf) {
    // Default parameters
    cg_dest_ = "./";

    // Load the model
    Model model;
    pinocchio::urdf::buildModel(model_urdf, model, true);

    // Convert to AD model
    model_ = new ADModel(model.cast<ADScalar>());
    data_ = new ADData(*model_);

    q_sx_ = casadi::SX::sym("q", model.nq);
    v_sx_ = casadi::SX::sym("v", model.nv);

    q_.resize(model.nq);
    for (Eigen::DenseIndex k = 0; k < model.nq; ++k) {
        q_[k] = q_sx_(k);
    }

    v_.resize(model.nv);
    for (Eigen::DenseIndex k = 0; k < model.nv; ++k) {
        v_[k] = v_sx_(k);
    }

    // Run forward kinematics with the model
    pinocchio::framesForwardKinematics(*model_, *data_, q_);
}

int CodeGenerator::GenerateInertiaMatrix() {
    Eigen::Matrix<ADScalar, -1, -1> M(model_->nv, model_->nv);
    M = pinocchio::crba(*model_, *data_, q_);
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

    casadi::SX cs_M;
    pinocchio::casadi::copy(M, cs_M);
    GenerateCode("mass_matrix", casadi::SXVector{q_sx_}, casadi::SXVector{densify(cs_M)});
    return 0;
}

int CodeGenerator::GenerateBiasVector() {
    TangentVectorAD h(model_->nv);
    h = pinocchio::rnea(*model_, *data_, q_, v_, TangentVectorAD::Zero(model_->nv));
    casadi::SX cs_h;
    pinocchio::casadi::copy(h, cs_h);
    GenerateCode("bias_vector", casadi::SXVector{q_sx_, v_sx_}, casadi::SXVector{densify(cs_h)});
    return 0;
}

int CodeGenerator::GenerateEndEffectorData(const std::string &name,
                                           const std::string &parent_joint, const std::string &parent_frame,
                                           const Eigen::Vector3d &r, const Eigen::Matrix3d &R) {
    Eigen::Matrix<ADScalar, 3, 3> R_ad = R.cast<ADScalar>();
    Eigen::Matrix<ADScalar, 3, 1> r_ad = r.cast<ADScalar>();
    
    // Create frame on model
    model_->addFrame(pinocchio::FrameTpl<ADScalar>(name, model_->getJointId(parent_joint),
                                      model_->getFrameId(parent_frame), 
                                      pinocchio::SE3Tpl<ADScalar>(R_ad, r_ad),
                                      pinocchio::OP_FRAME));

    // Update data
    delete data_;
    data_ = new ADData(*model_);
    pinocchio::framesForwardKinematics(*model_, *data_, q_);
    
    ADData::Matrix6x J(6, model_->nv), dJdt(6, model_->nv);
    pinocchio::forwardKinematics(*model_, *data_, q_, v_, TangentVectorAD::Zero(model_->nv));
    pinocchio::updateFramePlacements(*model_, *data_);
    pinocchio::computeJointJacobians(*model_, *data_, q_);

    J.setZero();
    dJdt.setZero();
    pinocchio::getFrameJacobian(*model_, *data_, model_->getFrameId(name), pinocchio::LOCAL_WORLD_ALIGNED, J);
    pinocchio::MotionTpl<ADScalar> a = pinocchio::getFrameAcceleration(*model_, *data_, model_->getFrameId(name), pinocchio::LOCAL_WORLD_ALIGNED);
    // Create function
    casadi::SX cs_p, cs_J, cs_Jdot_qdot;
    pinocchio::casadi::copy(data_->oMf[model_->getFrameId(name)].translation(), cs_p);
    pinocchio::casadi::copy(J.topRows(3), cs_J);
    pinocchio::casadi::copy(a.linear(), cs_Jdot_qdot);

    // Generate code
    GenerateCode(name,
                 casadi::SXVector{q_sx_, v_sx_},
                 casadi::SXVector{densify(cs_p), densify(cs_J), densify(cs_Jdot_qdot)});

    return 0;
}

int CodeGenerator::GenerateCode(const std::string &name,
                                const casadi::SXVector &in,
                                const casadi::SXVector &out) {
    casadi::Dict opts;
    opts["with_header"] = true;
    casadi::Function fun = casadi::Function(model_->name + "_" + name, in, out);
    casadi::CodeGenerator cg(fun.name(), opts);
    cg.add(fun);
    cg.generate(cg_dest_);

    return 0;
}