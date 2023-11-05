#ifndef CASSIE_CONTROLLER_OS_CTRL_HPP
#define CASSIE_CONTROLLER_OS_CTRL_HPP

#include <glog/logging.h>

#include "controllers/model.h"
#include "controllers/osc/osc.h"
#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"
#include "model/cg/arm_actuation_matrix.h"
#include "model/cg/arm_bias_vector.h"
#include "model/cg/arm_constraint.h"
#include "model/cg/arm_mass_matrix.h"
#include "model/cg/arm_tip.h"

#define ARM_MODEL_NQ 3
#define ARM_MODEL_NV 3
#define ARM_MODEL_NU 3

using namespace controller;

class ArmModel : public DynamicModel {
   public:
    ArmModel() : DynamicModel(DynamicModel::Size(ARM_MODEL_NQ, ARM_MODEL_NV, ARM_MODEL_NU)) {
    }

    int MapMujocoState(const double* q, const double* v);

    // Tasks
    static void TipPositionTask(const ConfigurationVector& q, const TangentVector& v,
                                Vector& x, Matrix& J, Vector& dJdq) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {x.data(), J.data(), dJdq.data()};
        arm_tip(in, out, NULL, NULL, 0);
    }

    // Constraints
    static void WristAngleTask(const ConfigurationVector& q, const TangentVector& v,
                               Vector& x, Matrix& J, Vector& dJdq) {
        x << q[2] - 0.0;
        J << 0, 0, 1.0;
        dJdq << 0;
    }

   protected:
    // Dynamics

    void ComputeMassMatrix(const ConfigurationVector& q, Matrix& M) {
        const double* in[] = {q.data(), NULL};
        double* out[] = {M.data()};
        arm_mass_matrix(in, out, NULL, NULL, 0);
    }

    void ComputeBiasVector(const ConfigurationVector& q, const TangentVector& v, Vector& h) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {h.data()};
        arm_bias_vector(in, out, NULL, NULL, 0);
    }

    void ComputeActuationMap(const ConfigurationVector& q, Matrix& B) {
        const double* in[] = {q.data()};
        double* out[] = {B.data()};
        arm_actuation_matrix(in, out, NULL, NULL, 0);
    }
};

class ArmOSC : public controller::osc::OperationalSpaceController {
   public:
    ArmOSC() : controller::osc::OperationalSpaceController() {
        // Create arm model
        m_ = new ArmModel();
        // Add model to controller
        AddModel(*m_);

        // Create tasks
        AddTask("tip_position", 3, &ArmModel::TipPositionTask);
        GetTask("tip_position")->SetTaskWeighting(1.0);
        GetTask("tip_position")->SetErrorGains(Vector(1.0, 1.0, 1.0), Vector(0.0, 0.0, 0.0));

        AddTask("wrist_angle", 1, &ArmModel::WristAngleTask);
        GetTask("wrist_angle")->SetTaskWeighting(1.0);
        GetTask("wrist_angle")->SetErrorGains(Vector(1.0), Vector(0.0));

        // Create constraints

        // Create controller
        CreateOSC();
    }

    ~ArmOSC() { delete m_; }

    void UpdateReferences(Scalar t, ConfigurationVector& q, TangentVector& v) {
        GetTask("name")->SetReference(q);
    }

   private:
    ArmModel* m_;
};

#endif /* CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP */
