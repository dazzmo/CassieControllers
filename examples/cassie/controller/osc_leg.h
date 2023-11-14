#ifndef CASSIE_CONTROLLER_OSC_LEG_HPP
#define CASSIE_CONTROLLER_OSC_LEG_HPP

#include <glog/logging.h>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"
// Code-generated functions
#include "model/cg/leg/cassie_actuation.h"
#include "model/cg/leg/cassie_ankle.h"
#include "model/cg/leg/cassie_bias_vector.h"
#include "model/cg/leg/cassie_foot_back.h"
#include "model/cg/leg/cassie_foot_front.h"
#include "model/cg/leg/cassie_heel_spring_constraint.h"
#include "model/cg/leg/cassie_mass_matrix.h"
#include "model/cg/leg/cassie_spring_forces.h"
// OSC model include
#include "controllers/osc/model.h"
#include "controllers/osc/tasks/joint_track_task.h"

#define CASSIE_LEG_NQ (8)
#define CASSIE_LEG_NV (8)
#define CASSIE_LEG_NU (5)

using namespace controller;

class CassieLegOSC : public osc::Model {
   public:
    CassieLegOSC() : osc::Model(DynamicModel::Size(CASSIE_LEG_NQ, CASSIE_LEG_NV, CASSIE_LEG_NU)) {
        initial_state().q << 0.00449956, 0, 0.497301, -1.1997, 0, 1.42671, 0.0, -1.59681;
        bounds().qmin << -0.3927, -0.3927, -0.8727, -2.8623, -0.3, 0.75, -0.3, -2.4435;
        bounds().qmax << 0.3927, 0.3927, 1.3963, -0.95, 0.3, 3.0, 0.3, -0.5236;
        bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9;
        bounds().vmax.setConstant(1e1);
        bounds().amax.setConstant(1e20);

        // Add ankle tracking task
        AddTask("ankle", 3, &CassieLegOSC::AnklePositionTask);
        GetTask("ankle")->SetTaskWeightMatrix(Vector3(1.0, 1.0, 1.0));
        GetTask("ankle")->SetKpGains(Vector3(1e2, 1e2, 1e2));
        GetTask("ankle")->SetKdGains(Vector3(1e0, 1e0, 1e0));

        // Add constraint
        AddHolonomicConstraint("rigid bar", 1, &CassieLegOSC::RigidBarConstraint);
    }
    ~CassieLegOSC() = default;

    // Function that gets called every time control is updated
    void UpdateReferences(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
        // GetTask("tip")->SetReference(Vector3(0.0, 1.0, -1.0));
        // GetTask("joint track")->SetReference(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));
    }

    void UpdateState(Dimension nq, const Scalar* q, Dimension nv, const Scalar* v);

    int HeelSpringDeflection();
    int InverseKinematics(Eigen::VectorXd& qpos, const Eigen::Vector3d& x_d, const Eigen::VectorXd& q0);

   protected:
    // Tasks
    static void AnklePositionTask(const ConfigurationVector& q, const TangentVector& v,
                                  Vector& x, Matrix& J, Vector& dJdt_v) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_ankle(in, out, NULL, NULL, 0);
    }

    // Constraints
    static void RigidBarConstraint(const ConfigurationVector& q, const TangentVector& v,
                                   Vector& c, Matrix& J, Vector& dJdt_v) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {c.data(), J.data(), dJdt_v.data(), nullptr};
        cassie_heel_spring_constraint(in, out, NULL, NULL, 0);
    }

    // Dynamics
    void ComputeMassMatrix(const ConfigurationVector& q, Matrix& M) {
        const double* in[] = {q.data(), NULL};
        double* out[] = {M.data()};
        cassie_mass_matrix(in, out, NULL, NULL, 0);
    }

    void ComputeBiasVector(const ConfigurationVector& q, const TangentVector& v, Vector& h) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {h.data()};
        cassie_bias_vector(in, out, NULL, NULL, 0);
    }

    void ComputeActuationMap(const ConfigurationVector& q, Matrix& B) {
        const double* in[] = {q.data()};
        double* out[] = {B.data()};
        cassie_actuation(in, out, NULL, NULL, 0);
    }

   private:
    bool ik_restart_ = true;
};

#endif /* CASSIE_CONTROLLER_OSC_LEG_HPP */