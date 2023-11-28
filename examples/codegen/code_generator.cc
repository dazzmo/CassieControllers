#include "code_generator.h"

CodeGenerator::CodeGenerator(const std::string &model_urdf) {

    // Default parameters
    cg_dest_ = "./";

    // Load the model
    Model model;
    pinocchio::urdf::buildModel(model_urdf, model, true);

    // Convert to auto-diff (AD) model
    model_ = new ADModel(model.cast<ADScalar>());
    data_ = new ADData(*model_);

    q_sx_ = casadi::SX::sym("q", model.nq);
    v_sx_ = casadi::SX::sym("v", model.nv);

    q_.resize(model.nq);
    for (int k = 0; k < model.nq; ++k) {
        q_[k] = q_sx_(k);
    }

    v_.resize(model.nv);
    for (int k = 0; k < model.nv; ++k) {
        v_[k] = v_sx_(k);
    }

    // Run forward kinematics and compute position of each frame
    pinocchio::framesForwardKinematics(*model_, *data_, q_);
}

// Compute mass matrix: get upper triangular part, fill the rest
int CodeGenerator::GenerateInertiaMatrix() {
    Eigen::Matrix<ADScalar, -1, -1> M(model_->nv, model_->nv);
    M = pinocchio::crba(*model_, *data_, q_);
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

    casadi::SX cs_M;
    pinocchio::casadi::copy(M, cs_M);
    GenerateCode("mass_matrix", casadi::SXVector{q_sx_}, casadi::SXVector{densify(cs_M)});
    return 0;
}

// Compute bias vector with Recursive Newton-Euler Algorithm 
// Gives inverse-dynamics torques from current state, so set acceleration to 0
int CodeGenerator::GenerateBiasVector() {
    TangentVectorAD h(model_->nv);
    h = pinocchio::rnea(*model_, *data_, q_, v_, TangentVectorAD::Zero(model_->nv));
    casadi::SX cs_h;
    pinocchio::casadi::copy(h, cs_h);
    GenerateCode("bias_vector", casadi::SXVector{q_sx_, v_sx_}, casadi::SXVector{densify(cs_h)});
    return 0;
}

// Add a reference frame to the model
int CodeGenerator::AddReferenceFrame(const std::string &name,
                                     const std::string &parent_joint, 
                                     const std::string &parent_frame,
                                     const Eigen::Vector3d &r, 
                                     const Eigen::Matrix3d &R) {
    // Cast frame data to AD
    Eigen::Matrix<ADScalar, 3, 3> R_ad = R.cast<ADScalar>();
    Eigen::Matrix<ADScalar, 3, 1> r_ad = r.cast<ADScalar>();

    // Create frame on model
    model_->addFrame(pinocchio::FrameTpl<ADScalar>(
        name, model_->getJointId(parent_joint),
        model_->getFrameId(parent_frame),
        pinocchio::SE3Tpl<ADScalar>(R_ad, r_ad),
        pinocchio::OP_FRAME)
    );

    // Update data
    delete data_;
    data_ = new ADData(*model_);

    return 0;
}

int CodeGenerator::GenerateEndEffectorData(const std::string &name,
                                           const std::string &parent_joint, const std::string &parent_frame,
                                           const Eigen::Vector3d &r, const Eigen::Matrix3d &R) {
    
    // Add a reference frame
    AddReferenceFrame(name, parent_joint, parent_frame, r, R);
    
    // Perform forward kinematics and compute frames
    pinocchio::framesForwardKinematics(*model_, *data_, q_);

    // Initiaise point Jacobian (with acceleration set to 0)
    ADData::Matrix6x J(6, model_->nv);
    ADData::Matrix6x dJdt(6, model_->nv);
    TangentVectorAD q_acc_zero = TangentVectorAD::Zero(model_->nv);

    pinocchio::forwardKinematics(*model_, *data_, q_, v_, q_acc_zero);
    pinocchio::updateFramePlacements(*model_, *data_);
    pinocchio::computeJointJacobians(*model_, *data_, q_);

    // Compute Jacobian and its time-derivative
    J.setZero();
    dJdt.setZero();
    pinocchio::getFrameJacobian(*model_, *data_, model_->getFrameId(name), pinocchio::LOCAL_WORLD_ALIGNED, J);
    pinocchio::MotionTpl<ADScalar> a = pinocchio::getFrameAcceleration(*model_, *data_, model_->getFrameId(name), pinocchio::LOCAL_WORLD_ALIGNED);

    // Create functions for point Jacobians (only position, so top 3 rows)
    // Note that dJdt*q_d = x_dd (linear acceleration) when q_dd = 0
    casadi::SX cs_p, cs_J, cs_dJdt_v;
    pinocchio::casadi::copy(data_->oMf[model_->getFrameId(name)].translation(), cs_p);
    pinocchio::casadi::copy(J.topRows(3), cs_J);
    pinocchio::casadi::copy(a.linear(), cs_dJdt_v);

    // Generate code
    GenerateCode(name,
                 casadi::SXVector{q_sx_, v_sx_},
                 casadi::SXVector{densify(cs_p), densify(cs_J), densify(cs_dJdt_v)});

    return 0;
}

// Generate code
int CodeGenerator::GenerateCode(const std::string &name,
                                const casadi::SXVector &in,
                                const casadi::SXVector &out) {
    casadi::Dict opts;
    opts["with_header"] = true;
    casadi::Function fun = casadi::Function(model_->name + "_" + name, in, out);
    std::cout << fun.name() << '\n';
    casadi::CodeGenerator cg(fun.name(), opts);
    cg.add(fun);
    cg.generate(cg_dest_);

    std::cout << "Finished\n";

    return 0;
}