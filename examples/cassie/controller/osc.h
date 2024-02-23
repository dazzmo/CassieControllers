#ifndef CONTROLLER_OSC_H
#define CONTROLLER_OSC_H

#include <glog/logging.h>
#include <math.h>

#include "eigen3/Eigen/Cholesky"
#include "eigen3/Eigen/Dense"

// Code-generated functions
#include "model/cg/cassie/cassie_LeftFootBack.h"
#include "model/cg/cassie/cassie_LeftFootFront.h"
#include "model/cg/cassie/cassie_RightFootBack.h"
#include "model/cg/cassie/cassie_RightFootFront.h"
#include "model/cg/cassie/cassie_achilles_rod_constraints.h"
#include "model/cg/cassie/cassie_actuation_map.h"
#include "model/cg/cassie/cassie_bias_vector.h"
#include "model/cg/cassie/cassie_centre_of_mass_data.h"
#include "model/cg/cassie/cassie_mass_matrix.h"

// OSC model
#include "controllers/osc/model.h"
#include "controllers/osc/tasks/joint_track_task.h"

// Sizes
#define CASSIE_NQ (23)
#define CASSIE_NV (22)
#define CASSIE_NU (10)

using namespace controller;

class CassieOSC : public osc::Model {
   public:
    CassieOSC()
        : osc::Model(DynamicModel::Size(CASSIE_NQ, CASSIE_NV, CASSIE_NU)) {
        // Set relevannt bounds
        bounds().umax << 4.5, 4.5, 12.2, 12.2, 0.9, 4.5, 4.5, 12.2, 12.2, 0.9;
        bounds().amax.setConstant(1e4);  // This is a guess

        // Task weights
        Eigen::Vector<Scalar, CASSIE_NU> ctrl_weights;
        Vector3 com_weights;
        Vector3 contact_weights;

        ctrl_weights.setConstant(1e-3);
        com_weights.setConstant(1e1);
        contact_weights.setConstant(1e2);

        // Gains
        // TODO: Add for centre of mass!
        double contact_kp = 1e2;
        double contact_kd = 1e1;

        // Set control weights in cost function
        SetControlWeighting(ctrl_weights);

        // Add task for foot contact points
        AddEndEffectorTask("right_foot_back", &CassieOSC::RightFootBackTask);
        GetEndEffectorTask("right_foot_back")
            ->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("right_foot_back")
            ->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("right_foot_back")
            ->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        AddEndEffectorTask("right_foot_front", &CassieOSC::RightFootFrontTask);
        GetEndEffectorTask("right_foot_front")
            ->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("right_foot_front")
            ->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("right_foot_front")
            ->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        AddEndEffectorTask("left_foot_back", &CassieOSC::LeftFootBackTask);
        GetEndEffectorTask("left_foot_back")
            ->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("left_foot_back")
            ->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("left_foot_back")
            ->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        AddEndEffectorTask("left_foot_front", &CassieOSC::LeftFootFrontTask);
        GetEndEffectorTask("left_foot_front")
            ->SetTaskWeightMatrix(contact_weights);
        GetEndEffectorTask("left_foot_front")
            ->SetKpGains(contact_kp * Eigen::VectorXd::Ones(3));
        GetEndEffectorTask("left_foot_front")
            ->SetKdGains(contact_kd * Eigen::VectorXd::Ones(3));

        // Add task for center of mass
        AddTask("com", 3, &CassieOSC::CenterOfMassTask);
        GetTask("com")->SetTaskWeightMatrix(com_weights);
        GetTask("com")->SetKpGains(Eigen::Vector3d(1e2, 1e2, 1e2));
        GetTask("com")->SetKdGains(Eigen::Vector3d(1e1, 1e1, 1e1));

        // NOTE: Joint damping task not configured for nq != nv. Add it when ready

        // Add kinematic constraint
        AddHolonomicConstraint("rigid bar", 6, &CassieOSC::RigidBarConstraint);
    }
    ~CassieOSC() = default;

    // Update the references for any tasks
    void UpdateReferences(Scalar time, const ConfigurationVector &q,
                          const TangentVector &v) {
        // TODO: Contact not currently working??
        // Make all points in contact
        GetEndEffectorTask("right_foot_front")->inContact = true;
        GetEndEffectorTask("right_foot_back")->inContact = true;
        GetEndEffectorTask("left_foot_front")->inContact = true;
        GetEndEffectorTask("left_foot_back")->inContact = true;

        // Get current position of the feet and set that as the reference
        // Update tasks
        GetEndEffectorTask("right_foot_front")->Update(state().q, state().v);
        GetEndEffectorTask("right_foot_back")->Update(state().q, state().v);
        GetEndEffectorTask("left_foot_front")->Update(state().q, state().v);
        GetEndEffectorTask("left_foot_back")->Update(state().q, state().v);

        // Fix contact positions to these positions
        GetEndEffectorTask("right_foot_front")
            ->SetReference(Eigen::Vector3d(0.08, -0.135, 0));
        GetEndEffectorTask("right_foot_back")
            ->SetReference(Eigen::Vector3d(-0.08, -0.135, 0));
        GetEndEffectorTask("left_foot_front")
            ->SetReference(Eigen::Vector3d(0.08,  0.135, 0));
        GetEndEffectorTask("left_foot_back")
            ->SetReference(Eigen::Vector3d(-0.08,  0.135, 0));

        GetTask("com")->SetReference(Eigen::Vector3d(-0.01656, 0.0, 0.87)); //0.7 - 0.1 * (time / 100.0)));

        // Set their positions to stay at
        LOG(INFO) << "Left front position: "
                  << GetEndEffectorTask("left_foot_front")->x().transpose();
        LOG(INFO) << "Left back position: "
                  << GetEndEffectorTask("left_foot_back")->x().transpose();
        LOG(INFO) << "Right front position: "
                  << GetEndEffectorTask("right_foot_front")->x().transpose();
        LOG(INFO) << "Right back position: "
                  << GetEndEffectorTask("right_foot_back")->x().transpose();

        LOG(INFO) << "Center of mass position: "
                  << GetTask("com")->x().transpose();
    }

    // Update controller state
    void UpdateState(Dimension nq, const Scalar *q, Dimension nv,
                     const Scalar *v);

   protected:
    // Tasks
    static void LeftFootBackTask(const ConfigurationVector &q,
                                 const TangentVector &v, Vector &x, Matrix &J,
                                 Vector &dJdt_v) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_LeftFootBack(in, out, NULL, NULL, 0);
    }

    static void LeftFootFrontTask(const ConfigurationVector &q,
                                  const TangentVector &v, Vector &x, Matrix &J,
                                  Vector &dJdt_v) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_LeftFootFront(in, out, NULL, NULL, 0);
    }

    static void RightFootBackTask(const ConfigurationVector &q,
                                  const TangentVector &v, Vector &x, Matrix &J,
                                  Vector &dJdt_v) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_RightFootBack(in, out, NULL, NULL, 0);
    }

    static void RightFootFrontTask(const ConfigurationVector &q,
                                   const TangentVector &v, Vector &x, Matrix &J,
                                   Vector &dJdt_v) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_RightFootFront(in, out, NULL, NULL, 0);
    }

    static void CenterOfMassTask(const ConfigurationVector &q,
                                 const TangentVector &v, Vector &x, Matrix &J,
                                 Vector &dJdt_v) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {x.data(), J.data(), dJdt_v.data()};
        cassie_centre_of_mass_data(in, out, NULL, NULL, 0);
    }

    // Constraints
    static void RigidBarConstraint(const ConfigurationVector &q,
                                   const TangentVector &v, Vector &c, Matrix &J,
                                   Vector &dJdt_v) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {c.data(), J.data(), dJdt_v.data(), nullptr};
        cassie_achilles_rod_constraints(in, out, NULL, NULL, 0);
    }

    // Dynamics
    void ComputeMassMatrix(const ConfigurationVector &q, Matrix &M) {
        const double *in[] = {q.data(), NULL};
        double *out[] = {M.data()};
        cassie_mass_matrix(in, out, NULL, NULL, 0);
    }

    void ComputeBiasVector(const ConfigurationVector &q, const TangentVector &v,
                           Vector &h) {
        const double *in[] = {q.data(), v.data()};
        double *out[] = {h.data()};
        cassie_bias_vector(in, out, NULL, NULL, 0);
    }

    void ComputeActuationMap(const ConfigurationVector &q, Matrix &B) {
        const double *in[] = {q.data()};
        double *out[] = {B.data()};
        cassie_actuation_map(in, out, NULL, NULL, 0);
    }

   private:
    osc::JointTrackTask *joint_track_task_;
};

#endif /* CONTROLLER_OSC_H */
