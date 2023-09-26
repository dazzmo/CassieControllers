#ifndef CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP
#define CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP
#ifndef CASSIE_CONTROLLER_OS_CTRL_HPP
#define CASSIE_CONTROLLER_OS_CTRL_HPP

#include <glog/logging.h>

#include "controllers/os_ctrl.h"
#include "eigen3/Eigen/Dense"
#include "model/cg/leg/cassie_actuation.h"
#include "model/cg/leg/cassie_bias_vector.h"
#include "model/cg/leg/cassie_foot_back.h"
#include "model/cg/leg/cassie_foot_front.h"
#include "model/cg/leg/cassie_heel_spring_constraint.h"
#include "model/cg/leg/cassie_heel_spring_constraint_inertia.h"
#include "model/cg/leg/cassie_mass_matrix.h"
#include "model/cg/leg/cassie_mass_matrix_inv.h"
#include "model/cg/leg/cassie_spring_forces.h"

class CassieLegOSController : public OperationalSpaceController {
   public:
    CassieLegOSController() : OperationalSpaceController(8, 8, 5) {
        LOG(INFO) << "CassieLegOSControlle::CassieLegOSController()";
        ctrl_max() << 10.0, 10.0, 12.2, 12.2, 0.9;
        SetupEndEffectors();
        InitMatrices();
        InitProgram();
    }
    ~CassieLegOSController() = default;

    int SetupEndEffectors();
    void UpdateDynamics();

    int HeelSpringDeflection();

    int MapMujocoState(const double* q, const double* v);

   protected:
   private:
    double c_;
    Eigen::MatrixXd N_;
    Eigen::MatrixXd J_;
    Eigen::MatrixXd dJdq_;
    Eigen::MatrixXd JMJT_;
    Eigen::MatrixXd M_;
    Eigen::MatrixXd Minv_;
    Eigen::MatrixXd h_;
    Eigen::MatrixXd h_spring_;
    Eigen::MatrixXd B_;

    int InitMatrices();
};

#endif /* CASSIE_CONTROLLER_OS_CTRL_HPP */

#endif /* CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP */
