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

#include "model/cg/cassie/cassie_left_foot_back.h"
#include "model/cg/cassie/cassie_left_foot_front.h"
#include "model/cg/cassie/cassie_right_foot_back.h"
#include "model/cg/cassie/cassie_right_foot_front.h"

// OSC model
#include "controllers/osc/model.h"
#include "controllers/osc/tasks/joint_track_task.h"

// Sizes
#define CASSIE_NQ (23)
#define CASSIE_NV (22)
#define CASSIE_NU (10)

using namespace controller;

class CassieOSC : public osc::Model
{

public:
    CassieOSC() : osc::Model(DynamicModel::Size(CASSIE_NQ, CASSIE_NV, CASSIE_NU))
    {

        // Set relevannt bounds
        bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9,
                         4.5, 4.5, 12.2, 12.2, 0.9;
        bounds().amax.setConstant(1e4); // This is a guess

        // Task weights
        Eigen::Vector<Scalar, CASSIE_NU> ctrl_weights;
        Vector3 com_weights;
        Vector3 contact_weights;

        ctrl_weights.setConstant(1e-3 * 0);
        com_weights.setConstant(1e1 * 0);
        contact_weights.setConstant(1e3 * 0);

        // Gains
        // TODO: Add for centre of mass!
        double contact_kp = 50.0 * 0;
        double contact_kd = 5.0 * 0;

        // Set control weights in cost function
        SetControlWeighting(ctrl_weights);

        // TODO: Remove this in a sec
        AddTask("left_ankle", 3, &CassieOSC::LeftAnklePositionTask);
        GetTask("left_ankle")->SetTaskWeightMatrix(contact_weights * 0);

        // Add task for foot contact points
        AddEndEffectorTask("right_foot_back", &CassieOSC::RightFootBackTask);
        GetEndEffectorTask("right_foot_back")->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("right_foot_back")->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("right_foot_back")->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        AddEndEffectorTask("right_foot_front", &CassieOSC::RightFootFrontTask);
        GetEndEffectorTask("right_foot_front")->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("right_foot_front")->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("right_foot_front")->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        AddEndEffectorTask("left_foot_back", &CassieOSC::LeftFootBackTask);
        GetEndEffectorTask("left_foot_back")->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("left_foot_back")->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("left_foot_back")->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));
        
        AddEndEffectorTask("left_foot_front", &CassieOSC::LeftFootFrontTask);
        GetEndEffectorTask("left_foot_front")->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("left_foot_front")->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("left_foot_front")->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        // Add task for center of mass
        AddTask("com", 3, &CassieOSC::CenterOfMassTask);
        GetTask("com")->SetTaskWeightMatrix(com_weights);

        // NOTE: Joint damping task not configured for nq != nv. Add it when ready

        // Add kinematic constraint
        AddHolonomicConstraint("rigid bar", 6, &CassieOSC::RigidBarConstraint);
    }
    ~CassieOSC() = default;

    // Update the references for any tasks
    void UpdateReferences(Scalar time, const ConfigurationVector &q, const TangentVector &v)
    { 
        // TODO: Contact not currently working??
        // Make all points in contact
        GetEndEffectorTask("right_foot_front")->inContact = false;
        GetEndEffectorTask("right_foot_back")->inContact = false;
        GetEndEffectorTask("left_foot_front")->inContact = false;
        GetEndEffectorTask("left_foot_back")->inContact = false;

        GetEndEffectorTask("right_foot_front")->SetReference(Eigen::Vector3d(0, 0, 0));
        GetEndEffectorTask("right_foot_back")->SetReference(Eigen::Vector3d(0, 0, 0));
        GetEndEffectorTask("left_foot_front")->SetReference(Eigen::Vector3d(0, 0, 0));
        GetEndEffectorTask("left_foot_back")->SetReference(Eigen::Vector3d(0, 0, 0));

        // l_f: -0.0888996  -0.135032    2.01709
        // l_b: 0.0489675 -0.134511   1.90136
        // r_f: -0.0888996   0.135032    2.01709
        // r_b: 0.0489675  0.134511   1.90136
        // com: -0.0165602 -0.000103072      1.13837

        LOG(INFO) << "Left ankle position: " << GetTask("left_ankle")->x().transpose();

        // Set their positions to stay at
        LOG(INFO) << "Left front position: " << GetEndEffectorTask("left_foot_front")->x().transpose();
        LOG(INFO) << "Left back position: " << GetEndEffectorTask("left_foot_back")->x().transpose();
        LOG(INFO) << "Right front position: " << GetEndEffectorTask("right_foot_front")->x().transpose();
        LOG(INFO) << "Right back position: " << GetEndEffectorTask("right_foot_back")->x().transpose();

        LOG(INFO) << "Center of mass position: " << GetTask("com")->x().transpose();
    }

    // Update controller state
    void UpdateState(Dimension nq, const Scalar *q, Dimension nv, const Scalar *v);

protected:
    // Tasks
    static void LeftFootBackTask(const ConfigurationVector &q, const TangentVector &v,
                                 Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_left_foot_back(in, out, NULL, NULL, 0);
    }

    static void LeftFootFrontTask(const ConfigurationVector &q, const TangentVector &v,
                                 Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_left_foot_front(in, out, NULL, NULL, 0);
    }

    static void RightFootBackTask(const ConfigurationVector &q, const TangentVector &v,
                                 Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_right_foot_back(in, out, NULL, NULL, 0);
    }

    static void RightFootFrontTask(const ConfigurationVector &q, const TangentVector &v,
                                 Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_right_foot_front(in, out, NULL, NULL, 0);
    }

    static void LeftAnklePositionTask(const ConfigurationVector &q, const TangentVector &v,
                                      Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_left_ankle(in, out, NULL, NULL, 0);
    }

    static void RightAnklePositionTask(const ConfigurationVector &q, const TangentVector &v,
                                       Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_right_ankle(in, out, NULL, NULL, 0);
    }

    static void CenterOfMassTask(const ConfigurationVector &q, const TangentVector &v,
                                 Vector &x, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_centre_of_mass_data(in, out, NULL, NULL, 0);
    }

    // Constraints
    static void RigidBarConstraint(const ConfigurationVector &q, const TangentVector &v,
                                   Vector &c, Matrix &J, Vector &dJdt_v)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {c.data(), J.data(), dJdt_v.data(), nullptr};
        cassie_achilles_rod_constraints(in, out, NULL, NULL, 0);
    }

    // Dynamics
    void ComputeMassMatrix(const ConfigurationVector &q, Matrix &M)
    {
        const double *in[] = {q.data(), NULL};
        double *out[] = {M.data()};
        cassie_mass_matrix(in, out, NULL, NULL, 0);
    }

    void ComputeBiasVector(const ConfigurationVector &q, const TangentVector &v, Vector &h)
    {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {h.data()};
        cassie_bias_vector(in, out, NULL, NULL, 0);
    }

    void ComputeActuationMap(const ConfigurationVector &q, Matrix &B)
    {
        const double *in[] = {q.data()};
        double *out[] = {B.data()};
        cassie_actuation_map(in, out, NULL, NULL, 0);
    }

private:
    osc::JointTrackTask *joint_track_task_;
};

#endif /* OSC_HPP */