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

    // Standard model?
    typedef double Scalar;
    typedef pinocchio::ModelTpl<Scalar> Model;
    typedef Model::Data Data;
    typedef Model::ConfigVectorType ConfigVector;
    typedef Model::TangentVectorType TangentVector;

    // Auto-diff model
    typedef casadi::SX ADScalar;
    typedef pinocchio::ModelTpl<ADScalar> ADModel;
    typedef ADModel::Data ADData;
    typedef ADModel::ConfigVectorType ConfigVectorAD;
    typedef ADModel::TangentVectorType TangentVectorAD;

    // Create model from Cassie urdf (just one leg)
    Model model;
    pinocchio::urdf::buildModel("cassie_leg.urdf", model, true);

    // Joint positions and velocities
    ConfigVector q(model.nq);
    TangentVector v(model.nv);

    // Get foot frame roation matrix
    // These angles come from here: https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model
    // TODO: Double-check the order (ZYX vs. XYZ)
    Eigen::AngleAxisd rollAngle(0.0, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(-140.0 * M_PI / 180.0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(-M_PI_2, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond foot_rot_q = yawAngle * pitchAngle * rollAngle;

    // The following sites are needed to add in the 4-bar linkage
    std::vector<BodySite> sites = {
        {   // https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model
            .name = "foot", 
            .parent_joint = "LeftFootPitch", 
            .parent_frame = "leftfoot", 
            .r = Eigen::Vector3d(0.01762, 0.05219, 0.0), 
            .R = foot_rot_q.matrix()
        },
        {   // https://github.com/agilityrobotics/cassie-doc/wiki/Heel-Spring-Model
            .name = "heel_tip", 
            .parent_joint = "LeftAchillesSpring", 
            .parent_frame = "leftheelspring", 
            .r = Eigen::Vector3d(0.11877, -0.01, 0.0), 
            .R = Eigen::Matrix3d::Identity()
        },
        {   // https://github.com/agilityrobotics/cassie-doc/wiki/Thigh-Model
            .name = "achilles_socket", 
            .parent_joint = "LeftHipPitch", 
            .parent_frame = "lefthippitch", 
            .r = Eigen::Vector3d(0.0, 0.0, 0.045), 
            .R = Eigen::Matrix3d::Identity()
        }
    };

    // Add each frame to the model
    for (BodySite& site : sites) {
        model.addFrame(pinocchio::Frame(
            site.name, 
            model.getJointId(site.parent_joint),
            model.getFrameId(site.parent_frame),
            pinocchio::SE3(site.R, site.r), 
            pinocchio::OP_FRAME)
        );
    }

    // Define contact locations
    // https://github.com/agilityrobotics/cassie-doc/wiki/Toe-Model
    std::vector<BodySite> contact_locations {
        {   
            .name = "foot_front",
            .parent_joint = "LeftFootPitch",
            .parent_frame = "foot",
            .r = Eigen::Vector3d(0.09, 0.0, 0.0),
            .R = Eigen::Matrix3d::Identity()
        },
        {
            .name = "foot_back", 
            .parent_joint = "LeftFootPitch", 
            .parent_frame = "foot", 
            .r = Eigen::Vector3d(-0.09, 0.0, 0.0), 
            .R = Eigen::Matrix3d::Identity()
        },
        {
            .name = "ankle",
            .parent_joint = "LeftFootPitch", 
            .parent_frame = "leftfoot", 
            .r = Eigen::Vector3d(0.0, 0.0, 0.0), 
            .R = Eigen::Matrix3d::Identity()
        }
    };

    // Add to model
    for (BodySite& site : contact_locations) {
        model.addFrame(pinocchio::Frame(
            site.name, 
            model.getJointId(site.parent_joint),
            model.getFrameId(site.parent_frame),
            pinocchio::SE3(site.R, site.r), 
            pinocchio::OP_FRAME)
        );
    }

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

    // Cast the model to the Casadi auto-diff model for codegen
    ADModel ad_model = model.cast<ADScalar>();
    ADData ad_data(ad_model);

    // Symbolic variables for joint positions and velocities
    ADScalar cs_q = ADScalar::sym("q", model.nq);
    ConfigVectorAD q_ad(model.nq);
    for (int k = 0; k < model.nq; ++k) {
        q_ad[k] = cs_q(k);
    }

    ADScalar cs_v = ADScalar::sym("v", model.nv);
    TangentVectorAD v_ad(model.nv);
    for (int k = 0; k < model.nv; ++k) {
        v_ad[k] = cs_v(k);
    }

    // Call forward kinematics in Pinocchio and compute position of each frame
    pinocchio::framesForwardKinematics(ad_model, ad_data, q_ad);

    // Achilles rod on Cassie constrains each end to be 501.2mm apart:
    // https://github.com/agilityrobotics/cassie-doc/wiki/Achilles-Rod-Model
    const double achilles_length = 0.5012;

    // Create constraint
    ADData::Vector3 dl = ad_data.oMf[ad_model.getFrameId("achilles_socket")].translation() -
                         ad_data.oMf[ad_model.getFrameId("heel_tip")].translation();
    ADScalar cl = dl.squaredNorm() - achilles_length * achilles_length;
    
    // Get Jacobian of constraint, its time-derivative, and the Hessian
    ADScalar Jcl = jacobian(cl, cs_q);
    ADScalar Hcl = hessian(cl, cs_q);
    ADScalar dJcldt = jacobian(mtimes(Jcl, cs_v), cs_q); // dJdt = dJdq * dqdt by chain rule

    // Compute mass matrix and bias forces
    TangentVectorAD h(model.nv);
    Eigen::Matrix<ADScalar, -1, -1> M(model.nv, model.nv);

    // Recursive Newton-Euler Algorithm gives inverse-dynamics torques given current state
    h = pinocchio::rnea(ad_model, ad_data, q_ad, v_ad, TangentVectorAD::Zero(model.nv)); 
    
    // Compute upper-triangular part of M and fill the rest
    M = pinocchio::crba(ad_model, ad_data, q_ad);
    M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();

    // Add to auto-diff model
    ADScalar cs_M, cs_h;
    pinocchio::casadi::copy(M, cs_M);
    pinocchio::casadi::copy(h, cs_h);

    // Spring dynamics parameters from:
    // https://github.com/jpreher/cassie_description/blob/master/MATLAB/Cassie_v4.m#L193
    // TODO: Check that these are actually similar to our own Cassie!
    ADScalar spring_forces = ADScalar::zeros(model.nv);
    double k_knee_stiffness = 2300.0;
    double k_heel_stiffness = 2000.0;
    double b_knee_damping = 4.6;
    double b_heel_damping = 4.0;

    // Extract joint coordinates of springs
    ADScalar q_knee = cs_q(model.joints[model.getJointId("LeftShinPitch")].idx_q());
    ADScalar q_heel = cs_q(model.joints[model.getJointId("LeftAchillesSpring")].idx_q());

    ADScalar v_knee = cs_v(model.joints[model.getJointId("LeftShinPitch")].idx_v());
    ADScalar v_heel = cs_v(model.joints[model.getJointId("LeftAchillesSpring")].idx_v());

    // Add to spring forces
    spring_forces(model.joints[model.getJointId("LeftShinPitch")].idx_v()) = k_knee_stiffness * (q_knee) + b_knee_damping * (v_knee);
    spring_forces(model.joints[model.getJointId("LeftAchillesSpring")].idx_v()) = k_heel_stiffness * (q_heel) + b_heel_damping * (v_heel);

    // Create the actuation/torque coefficient matrix
    // TODO: This is just the gear ratios again, don't hard-code it twice
    ADScalar B = ADScalar::zeros(model.nv, 5);
    B(model.joints[model.getJointId("LeftHipRoll")].idx_v(), 0) = 25.0;
    B(model.joints[model.getJointId("LeftHipYaw")].idx_v(), 1) = 25.0;
    B(model.joints[model.getJointId("LeftHipPitch")].idx_v(), 2) = 16.0;
    B(model.joints[model.getJointId("LeftKneePitch")].idx_v(), 3) = 16.0;
    B(model.joints[model.getJointId("LeftFootPitch")].idx_v(), 4) = 50.0;

    // Define functions to produce kinematics and constraint Jacobian
    std::vector<casadi::Function> functions {
        casadi::Function(model.name + "_mass_matrix", casadi::SXVector{cs_q}, casadi::SXVector{densify(cs_M)}),
        casadi::Function(model.name + "_bias_vector", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(cs_h)}),
        casadi::Function(model.name + "_spring_forces", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(spring_forces)}),
        casadi::Function(model.name + "_heel_spring_constraint", casadi::SXVector{cs_q, cs_v}, casadi::SXVector{densify(cl), densify(Jcl), densify(mtimes(dJcldt, cs_v)), densify(Hcl)}), // Usually want dJdt*dqdt = dJdq*dqdt*dqdt
        casadi::Function(model.name + "_actuation", casadi::SXVector{cs_q}, casadi::SXVector{densify(B)})
    };

    // Define end-effector positions and Jacobian
    ADData::Matrix6x J(6, model.nv);
    ADData::Matrix6x dJdt(6, model.nv);
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad, v_ad, TangentVectorAD::Zero(model.nv));
    pinocchio::updateFramePlacements(ad_model, ad_data);
    pinocchio::computeJointJacobians(ad_model, ad_data, q_ad);

    // Create functions for each contact point
    for (BodySite& site : contact_locations) {

        // Site Jacobian
        J.setZero();
        pinocchio::getFrameJacobian(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED, J);

        // Acceleration of site
        pinocchio::MotionTpl<ADScalar> a = pinocchio::getFrameAcceleration(ad_model, ad_data, ad_model.getFrameId(site.name), pinocchio::LOCAL_WORLD_ALIGNED);

        // Create functions for point Jacobians
        // Note that dJdt*q_d = x_dd (linear acceleration) when q_dd = 0
        ADScalar cs_p, cs_J, cs_dJdt_v;
        pinocchio::casadi::copy(ad_data.oMf[ad_model.getFrameId(site.name)].translation(), cs_p);
        pinocchio::casadi::copy(J.topRows(3), cs_J); // Top 3 rows are for point translation
        pinocchio::casadi::copy(a.linear(), cs_dJdt_v);

        functions.push_back(casadi::Function(
            model.name + "_" + site.name,
            casadi::SXVector{cs_q, cs_v},
            casadi::SXVector{densify(cs_p), densify(cs_J), densify(cs_dJdt_v)})
        );
    }

    // Generate code at requested file location
    casadi::Dict opts;
    opts["with_header"] = true;
    for (casadi::Function& fun : functions) {
        casadi::CodeGenerator cg(fun.name(), opts);
        cg.add(fun);
        cg.generate(argv[1]);
    }

    return 0;
}
