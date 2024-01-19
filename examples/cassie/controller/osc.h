#ifndef OSC_HPP
#define OSC_HPP

#include <glog/logging.h>
#include <math.h>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"

// Code-generated functions
#include "model/cg/cassie/cassie_achilles_rod_constraints.h"
#include "model/cg/cassie/cassie_actuation_map.h"
#include "model/cg/cassie/cassie_centre_of_mass_data.h"
#include "model/cg/cassie/cassie_left_ankle.h"
#include "model/cg/cassie/cassie_right_ankle.h"
#include "model/cg/cassie/cassie_bias_vector.h"
#include "model/cg/cassie/cassie_mass_matrix.h"

// OSC model
#include "controllers/osc/model.h"
#include "controllers/osc/tasks/joint_track_task.h"

// Sizes
#define CASSIE_NQ (16)
#define CASSIE_NV (16)
#define CASSIE_NU (10)

using namespace controller;

class CassieOSC : public osc::Model {

   public:
    CassieOSC() : osc::Model(DynamicModel::Size(CASSIE_NQ, CASSIE_NV, CASSIE_NU)) {

        // Order: hip roll, yaw, pitch, knee, shin (spring), tarsus, heel spring, toe
        initial_state().q <<  0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0, -1.5968,
                             -0.0045, 0.0, 0.4973, -1.1997, 0.0, 1.4267, 0.0, -1.5968;

        // Add bounds (from cassie.xml, different from kinematic model on wiki)
        bounds().qmin << -15.0, -22.5, -50.0, -164.0, -20.0,  50.0, -20.0, -140.0,
                         -15.0, -22.5, -50.0, -164.0, -20.0,  50.0, -20.0, -140.0; 
        bounds().qmax <<  22.5,  22.5,  80.0,  -37.0,  20.0, 170.0,  20.0,  -30.0,
                          22.5,  22.5,  80.0,  -37.0,  20.0, 170.0,  20.0,  -30.0;
        bounds().qmin *= M_PI /180;
        bounds().qmax *= M_PI /180;

        bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9,
                         4.5, 4.5, 12.2, 12.2, 0.9;
        bounds().vmax << 12.15, 12.15, 8.5, 8.5, 20, 20, 20, 11.52,
                         12.15, 12.15, 8.5, 8.5, 20, 20, 20, 11.52; // From cassie.urdf
        bounds().amax.setConstant(1e4);                             // This is a guess

        // Task weights
        Eigen::Vector<Scalar, CASSIE_NU> ctrl_weights;
        Eigen::Vector<Scalar, CASSIE_NQ> damp_weights;
        Vector3 ankle_weights;

        ctrl_weights.setConstant(1e-3);
        ankle_weights.setConstant(1e0);
        damp_weights.setConstant(1e-4);

        // PD weights: damping on all joints except springs
        double ankle_kp = 50.0;
        double ankle_kd = 5.0;
        
        Eigen::Vector<Scalar, CASSIE_NV> damp_kds;
        damp_kds.setConstant(10.0); 
        damp_kds[4] = 0.0;
        damp_kds[6] = 0.0;
        damp_kds[12] = 0.0;
        damp_kds[14] = 0.0;

        // Set control weights in cost function
        SetControlWeighting(ctrl_weights);

        // Add task for tracking ankles in 3D space
        AddTask("left ankle", 3, &CassieOSC::LeftAnklePositionTask);
        GetTask("left ankle")->SetTaskWeightMatrix(ankle_weights);
        GetTask("left ankle")->SetKpGains(Vector3(ankle_kp, ankle_kp, ankle_kp));
        GetTask("left ankle")->SetKdGains(Vector3(ankle_kd, ankle_kd, ankle_kd));

        AddTask("right ankle", 3, &CassieOSC::RightAnklePositionTask);
        GetTask("right ankle")->SetTaskWeightMatrix(ankle_weights);
        GetTask("right ankle")->SetKpGains(Vector3(ankle_kp, ankle_kp, ankle_kp));
        GetTask("right ankle")->SetKdGains(Vector3(ankle_kd, ankle_kd, ankle_kd));

        // Add task for center of mass with no weights, just for testing
        AddTask("com", 3, &CassieOSC::CenterOfMassTask);
        GetTask("com")->SetTaskWeightMatrix(ankle_weights*0);

        // Joint damping
        joint_track_task_ = new osc::JointTrackTask(this->size());
        AddTask("joint damp", std::shared_ptr<controller::osc::Task>(joint_track_task_));
        GetTask("joint damp")->SetTaskWeightMatrix(damp_weights);
        GetTask("joint damp")->SetKdGains(damp_kds);

        // Add kinematic constraint
        AddHolonomicConstraint("rigid bar", 6, &CassieOSC::RigidBarConstraint);
    }
    ~CassieOSC() = default;

    // Update the references for any tasks
    void UpdateReferences(Scalar time, const ConfigurationVector& q, const TangentVector& v) {
        double l_phase = -2.0*M_PI/4.0*time;
        double l_xpos = 0.0 + 0.2*cos(l_phase);
        double l_ypos = 0.1;
        double l_zpos = -0.7 + 0.2*sin(l_phase);

        double r_phase = -2.0*M_PI/4.0*time + M_PI_2;
        double r_xpos = 0.0 + 0.2*cos(r_phase);
        double r_ypos = -0.1;
        double r_zpos = -0.7 + 0.2*sin(r_phase);
        
        GetTask("left ankle")->SetReference(Vector3(l_xpos, l_ypos, l_zpos));
        GetTask("right ankle")->SetReference(Vector3(r_xpos, r_ypos, r_zpos));

        // LOG(INFO) << "Left ankle tracking error: " << GetTask("left ankle")->e().transpose();
        // LOG(INFO) << "Right ankle tracking error: " << GetTask("right ankle")->e().transpose();

        LOG(INFO) << "Center of mass position: " << GetTask("com")->x().transpose();
    }

    // Update controller state
    void UpdateState(Dimension nq, const Scalar* q, Dimension nv, const Scalar* v);

   protected:

    // Tasks
    static void LeftAnklePositionTask(const ConfigurationVector& q, const TangentVector& v,
                                      Vector& x, Matrix& J, Vector& dJdt_v) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_left_ankle(in, out, NULL, NULL, 0);
    }

    static void RightAnklePositionTask(const ConfigurationVector& q, const TangentVector& v,
                                       Vector& x, Matrix& J, Vector& dJdt_v) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_right_ankle(in, out, NULL, NULL, 0);
    }

    static void CenterOfMassTask(const ConfigurationVector& q, const TangentVector& v,
                                 Vector& x, Matrix& J, Vector& dJdt_v) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_centre_of_mass_data(in, out, NULL, NULL, 0);
    }

    // Constraints
    static void RigidBarConstraint(const ConfigurationVector& q, const TangentVector& v,
                                   Vector& c, Matrix& J, Vector& dJdt_v) {
        const double* in[] = {q.data(), v.data()};
        double* out[] = {c.data(), J.data(), dJdt_v.data(), nullptr};
        cassie_achilles_rod_constraints(in, out, NULL, NULL, 0);
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


#endif /* OSC_HPP */