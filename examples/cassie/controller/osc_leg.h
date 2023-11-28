#ifndef CASSIE_CONTROLLER_OSC_LEG_HPP
#define CASSIE_CONTROLLER_OSC_LEG_HPP

#include <glog/logging.h>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"

// Code-generated functions
#include "model/cg/leg/cassie_actuation_map.h"
#include "model/cg/leg/cassie_ankle.h"
#include "model/cg/leg/cassie_bias_vector.h"
#include "model/cg/leg/cassie_foot_back.h"
#include "model/cg/leg/cassie_foot_front.h"
#include "model/cg/leg/cassie_achilles_rod_constraint.h"
#include "model/cg/leg/cassie_mass_matrix.h"

// OSC model include
#include "controllers/osc/model.h"
#include "controllers/osc/tasks/joint_limits_task.h"
#include "controllers/osc/tasks/joint_track_task.h"

#define CASSIE_LEG_NQ (8)
#define CASSIE_LEG_NV (8)
#define CASSIE_LEG_NU (5)

using namespace controller;

class CassieLegOSC : public osc::Model {
   public:
    CassieLegOSC() : osc::Model(DynamicModel::Size(CASSIE_LEG_NQ, CASSIE_LEG_NV, CASSIE_LEG_NU)) {

        // hip roll, hip yaw, hip pitch, knee, shin, tarsus, heel spring, toe
        initial_state().q << 0.00449956, 0, 0.497301, -1.1997, 0, 1.42671, 0.0, -1.59681;
        bounds().qmin << -0.3927, -0.3927, -0.8727, -2.8623, -0.3, 0.75, -0.3, -2.4435;
        bounds().qmax << 0.3927, 0.3927, 1.3963, -0.95, 0.3, 3.0, 0.3, -0.5236;
        bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9; // TODO: This is without gear ratio. Is ok?
        bounds().vmax.setConstant(1e2);
        bounds().amax.setConstant(1e20);

        // Add ankle tracking task
        // TODO: Choose weights
        AddTask("ankle", 3, &CassieLegOSC::AnklePositionTask);
        GetTask("ankle")->SetTaskWeightMatrix(Vector3(1e0, 1e0, 1e0));
        GetTask("ankle")->SetKpGains(Vector3(10, 10, 10));
        GetTask("ankle")->SetKdGains(Vector3(1, 1, 1));

        // Joint damping (NO CONTROL ON TOES CURRENTLY)
        joint_track_task_ = new osc::JointTrackTask(this->size());
        AddTask("joint track", std::shared_ptr<controller::osc::Task>(joint_track_task_));
        GetTask("joint track")->SetTaskWeightMatrix(Eigen::Vector<Scalar, CASSIE_LEG_NQ>(1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5));
        GetTask("joint track")->SetKpGains(Eigen::Vector<Scalar, CASSIE_LEG_NQ>(0, 0, 0, 0, 0, 0, 0, 0));
        GetTask("joint track")->SetKdGains(Eigen::Vector<Scalar, CASSIE_LEG_NV>(0, 0, 0, 0, 0, 0, 0, 0));

        // Add constraint
        AddHolonomicConstraint("rigid bar", 1, &CassieLegOSC::RigidBarConstraint);
    }
    ~CassieLegOSC() = default;

    // Function that gets called every time control is updated
    void UpdateReferences(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
        GetTask("ankle")->SetReference(Vector3(0.0, 0.2, -0.5));
        // std::cout << GetTask("ankle") -> x() << std::endl;

        // Compute IK for leg
        ConfigurationVector qd = q;
        InverseKinematics(qd, Vector3(0, 0.2, -0.5), q);
        LOG(INFO) << "qd: " << qd.transpose();
        LOG(INFO) << "qa: " << q.transpose();
        GetTask("joint track")->SetReference(qd);
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
        cassie_achilles_rod_constraint(in, out, NULL, NULL, 0);
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
        cassie_actuation_map(in, out, NULL, NULL, 0);
    }

   private:
    bool ik_restart_ = true;
    controller::osc::JointTrackTask* joint_track_task_;
    controller::osc::JointLimitsTask* joint_limits_task_;
};

#endif /* CASSIE_CONTROLLER_OSC_LEG_HPP */