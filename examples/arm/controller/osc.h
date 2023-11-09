#ifndef CASSIE_CONTROLLER_OS_CTRL_HPP
#define CASSIE_CONTROLLER_OS_CTRL_HPP

#include <glog/logging.h>

#include "controllers/osc/model.h"
#include "controllers/osc/osc.h"
#include "controllers/osc/tasks/joint_track_task.h"
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

class ArmModel : public osc::Model {
   public:
    ArmModel() : osc::Model(DynamicModel::Size(ARM_MODEL_NQ, ARM_MODEL_NV, ARM_MODEL_NU)) {
        // Model bounds
        state_init().q << 0.0, 0.0, 0.0;
        bounds().qmin << -M_PI, -M_PI, -M_PI;
        bounds().qmax << M_PI, M_PI, M_PI;
        bounds().umax << 20.0, 20.0, 20.0;
        bounds().vmax.setConstant(1e1);
        bounds().amax.setConstant(1e20);

        // Add tasks here

        AddTask("tip", 3, &ArmModel::TipPositionTask);
        GetTask("tip")->SetTaskWeighting(1.0);
        GetTask("tip")->SetErrorGains(Eigen::Vector<double, 3>(0.0, 1e2, 1e2), 
                                      Eigen::Vector<double, 3>(0.0, 1e0, 1e0));

        // Joint damping
        joint_track_task = new osc::JointTrackTask(this->size());
        GetTaskMap()["joint limit track"] = std::shared_ptr<controller::osc::Task>(joint_track_task);
        GetTask("joint limit track")->SetTaskWeighting(1e1);
        GetTask("joint limit track")->SetErrorGains(Eigen::Vector<double, 3>(0.0, 0.0, 0.0), Eigen::Vector<double, 3>(1e1, 1e1, 1e1));
    }

    // Function that gets called every time control is updated
    void UpdateReferences(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
        GetTask("tip")->SetReference(Vector3(0.0, -1.0, 1.0));
        GetTask("joint limit track")->SetReference(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));
    }
    
    // Tasks
    static void TipPositionTask(const ConfigurationVector& q, const TangentVector& v,
                                Vector& x, Matrix& J, Vector& Jdot_qdot) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {x.data(), J.data(), Jdot_qdot.data()};
        arm_tip(in, out, NULL, NULL, 0);
    }

    // Constraints
    static void WristAngleTask(const ConfigurationVector& q, const TangentVector& v,
                               Vector& x, Matrix& J, Vector& Jdot_qdot) {
        x << q[2] - 0.0;
        J << 0, 0, 1.0;
        Jdot_qdot << 0;
    }


   protected:
    // Dynamics
    void
    ComputeMassMatrix(const ConfigurationVector& q, Matrix& M) {
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

    osc::JointTrackTask* joint_track_task;
};

#endif /* CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP */
