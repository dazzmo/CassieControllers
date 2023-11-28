#include "code_generator.h"

int main(int argc, char* argv[]) {
    CodeGenerator cg("./arm.urdf");
    cg.SetCodeGenerationDestination(argv[1]);

    // Compute inertia matrix
    cg.GenerateInertiaMatrix();
    // Compute bias vector
    cg.GenerateBiasVector();

    // Task for tip
    cg.GenerateEndEffectorData("tip",
                               "wrist", "third_link",
                               Eigen::Vector3d(0, 0, -1.0),
                               Eigen::Matrix3d::Identity());

    CodeGenerator::ADModel& model = cg.GetModel();

    // Create actuation matrix (with gearing)
    casadi::SX B = casadi::SX::zeros(model.nv, 3);
    B(model.joints[model.getJointId("shoulder")].idx_v(), 0) = 1.0;
    B(model.joints[model.getJointId("elbow")].idx_v(), 1) = 1.0;
    B(model.joints[model.getJointId("wrist")].idx_v(), 2) = 1.0;

    cg.GenerateCode("actuation_matrix",
                    {cg.GetQposSX(), cg.GetQvelSX()},
                    {B});

    // Constraint for wrist and elbow
    casadi::SX c = cg.GetQposSX()(model.joints[model.getJointId("wrist")].idx_q());
    casadi::SX Jc = jacobian(c, cg.GetQposSX());
    casadi::SX dJcdq = mtimes(jacobian(mtimes(Jc, cg.GetQvelSX()), cg.GetQposSX()), cg.GetQvelSX());

    cg.GenerateCode("constraint",
                    {cg.GetQposSX(), cg.GetQvelSX()},
                    {c, densify(Jc), densify(dJcdq)});

    return 0;
}
