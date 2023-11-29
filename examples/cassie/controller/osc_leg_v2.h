#ifndef OSC_LEG_HPP
#define OSC_LEG_HPP

#include <glog/logging.h>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"

// Code-generated functions
#include "model/cg/leg/cassie_achilles_rod_constraint.h"
#include "model/cg/leg/cassie_actuation_map.h"
#include "model/cg/leg/cassie_ankle.h"
#include "model/cg/leg/cassie_bias_vector.h"
#include "model/cg/leg/cassie_mass_matrix.h"

// OSC model
#include "controllers/osc/model.h"
#include "controllers/osc/tasks/joint_track_task.h"

// Sizes
#define CASSIE_LEG_NQ (8)
#define CASSIE_LEG_NV (8)
#define CASSIE_LEG_NU (5)

using namespace controller;

class CassieLegOSC2 : public osc::Model {

   public:
    CassieLegOSC2() : osc::Model(DynamicModel::Size(CASSIE_LEG_NQ, CASSIE_LEG_NV, CASSIE_LEG_NU)) {

        // Order: hip roll, yaw, pitch, knee, shin (spring), tarsus, heel spring, toe
        initial_state().q <<  0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0, -1.5968;

        // Add bounds (from cassie.xml, different from kinematic model on wiki)
        bounds().qmin << -15.0, -22.5, -50.0, -164.0, -20.0,  50.0, -20.0, -140.0; 
        bounds().qmax <<  22.5,  22.5,  80.0,  -37.0,  20.0, 170.0,  20.0,  -30.0;
        bounds().qmin *= M_PI /180;
        bounds().qmax *= M_PI /180;

        bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9;
        bounds().vmax << 12.15, 12.15, 8.5, 8.5, 20, 20, 20, 11.52; // From cassie.urdf
        bounds().amax.setConstant(1e4);                             // This is a guess

        // Set control weights in cost function
        SetControlWeighting(Eigen::Vector<Scalar, CASSIE_LEG_NU>(1, 1, 1, 1, 1));

        // Add task for tracking ankle in 3D space
        AddTask("ankle", 3, &CassieLegOSC2::AnklePositionTask);
        GetTask("ankle")->SetTaskWeightMatrix(Vector3(1, 1, 1));
        GetTask("ankle")->SetKpGains(Vector3(0, 0, 0));
        GetTask("ankle")->SetKdGains(Vector3(0, 0, 0));

        // Joint damping
        joint_track_task_ = new osc::JointTrackTask(this->size());
        AddTask("joint damp", std::shared_ptr<controller::osc::Task>(joint_track_task_));
        GetTask("joint damp")->SetTaskWeightMatrix(Eigen::Vector<Scalar, CASSIE_LEG_NQ>(1, 1, 1, 1, 1, 1, 1, 1));
        GetTask("joint damp")->SetKdGains(Eigen::Vector<Scalar, CASSIE_LEG_NV>(0, 0, 0, 0, 0, 0, 0, 0));

        // Add kinematic constraint
        AddHolonomicConstraint("rigid bar", 1, &CassieLegOSC2::RigidBarConstraint);
        // AddProjectedConstraint("rigid bar", 1, &CassieLegOSC2::RigidBarConstraint);
    }
    ~CassieLegOSC2() = default;

    // Update the references for any tasks
    void UpdateReferences(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
        GetTask("ankle")->SetReference(Vector3(-0.020, 0.135, -0.8));
        // LOG(INFO) << "Ankle tracking PD error: " << GetTask("ankle")->ErrorOutputPD().transpose();
    }

    // Update controller state
    void UpdateState(Dimension nq, const Scalar* q, Dimension nv, const Scalar* v);

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
    osc::JointTrackTask* joint_track_task_;
};

#endif /* OSC_LEG_HPP */
