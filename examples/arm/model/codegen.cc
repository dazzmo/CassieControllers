#include "code_generator.h"

int main(int argc, char* argv[]) {
    CodeGenerator cg("./arm.urdf");
    cg.SetCodeGenerationDestination(argv[1]);

    cg.GenerateInertiaMatrix();
    cg.GenerateBiasVector();

    cg.GenerateEndEffectorData("tip",
                               "wrist", "third_link",
                               Eigen::Vector3d(0, 0, -1.0),
                               Eigen::Matrix3d::Identity());

    CodeGenerator::ADModel &model = cg.GetModel();
    
    // Create actuation matrix
    casadi::SX B = casadi::SX::zeros(model.nv, 3);
    B(model.joints[model.getJointId("shoulder")].idx_v(), 0) = 1.0;
    B(model.joints[model.getJointId("elbow")].idx_v(), 1) = 1.0;
    B(model.joints[model.getJointId("wrist")].idx_v(), 2) = 1.0;

    cg.GenerateCode("actuation_matrix",
                    {cg.GetConfigurationVectorSX(), cg.GetTangentVectorSX()},
                    {B});

    return 0;
}
