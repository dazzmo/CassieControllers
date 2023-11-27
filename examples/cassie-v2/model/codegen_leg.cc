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

    // Auto-diff model?
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

    // Define some sites on the leg that might be of interest
    std::vector<BodySites> sites = {
        
    };

    return 0;
}