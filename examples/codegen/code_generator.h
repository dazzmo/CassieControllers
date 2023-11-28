#ifndef EXAMPLES_CODEGEN_CODE_GENERATOR_HPP
#define EXAMPLES_CODEGEN_CODE_GENERATOR_HPP

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

class CodeGenerator {
   public:
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

    // Constructors and destructor
    CodeGenerator() = default;
    ~CodeGenerator() = default;

    CodeGenerator(const std::string &model_urdf);

    // Setters and Getters
    ADModel &GetModel() { return *model_; }
    ADData &GetData() { return *data_; }

    ConfigVectorAD &GetQpos() { return q_; }
    TangentVectorAD &GetQvel() { return v_; }

    casadi::SX &GetQposSX() { return q_sx_; }
    casadi::SX &GetQvelSX() { return v_sx_; }

    int GetJointIdq(const std::string &name) {return model_->joints[model_->getJointId(name)].idx_q(); }
    int GetJointIdv(const std::string &name) {return model_->joints[model_->getJointId(name)].idx_v(); }

    void SetCodeGenerationDestination(const std::string &dest) { cg_dest_ = dest; }

    // Kinematics

    // Dynamics
    int GenerateInertiaMatrix();
    int GenerateBiasVector();
    int AddReferenceFrame(const std::string &name,
                          const std::string &parent_joint, const std::string &parent_frame,
                          const Eigen::Vector3d &r, const Eigen::Matrix3d &R);
    int GenerateEndEffectorData(const std::string &name,
                                const std::string &parent_joint, const std::string &parent_frame,
                                const Eigen::Vector3d &r, const Eigen::Matrix3d &R);

    // Code generation

    /**
     * @brief Generates C code (with header) for the inputs and outputs
     * provided by in and out respectively. Code is output to the destination
     * provided by SetCodeGenerationDestination().
     *
     * @param name
     * @param in
     * @param out
     * @return int
     */
    int GenerateCode(const std::string &name,
                     const casadi::SXVector &in,
                     const casadi::SXVector &out);

   private:
    ADModel *model_;
    ADData *data_;

    ConfigVectorAD q_;
    TangentVectorAD v_;

    casadi::SX q_sx_;
    casadi::SX v_sx_;

    std::string model_name_;
    std::string cg_dest_;
};

#endif /* EXAMPLES_CODEGEN_CODE_GENERATOR_HPP */
