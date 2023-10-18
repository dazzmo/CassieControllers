#ifndef CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP
#define CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP
#ifndef CASSIE_CONTROLLER_OS_CTRL_HPP
#define CASSIE_CONTROLLER_OS_CTRL_HPP

#include <glog/logging.h>

#include "controllers/os_ctrl.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Cholesky"
#include "model/cg/leg/cassie_actuation.h"
#include "model/cg/leg/cassie_bias_vector.h"
#include "model/cg/leg/cassie_foot_back.h"
#include "model/cg/leg/cassie_foot_front.h"
#include "model/cg/leg/cassie_ankle.h"
#include "model/cg/leg/cassie_heel_spring_constraint.h"
#include "model/cg/leg/cassie_mass_matrix.h"
#include "model/cg/leg/cassie_spring_forces.h"

class CassieLegOSController : public OperationalSpaceController {
   public:
    CassieLegOSController() : OperationalSpaceController(8, 8, 5) {
        LOG(INFO) << "CassieLegOSControlle::CassieLegOSController()";
        ctrl_max() << 4.5, 4.5, 12.2, 12.2, 0.9;
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
    Eigen::VectorXd dJdq_;
    Eigen::MatrixXd JMJT_;
    Eigen::MatrixXd M_;
    Eigen::MatrixXd Minv_;
    Eigen::VectorXd h_;
    Eigen::VectorXd h_spring_;
    Eigen::MatrixXd B_;

    bool first_solve_ = true;

    int InitMatrices();
};

#endif /* CASSIE_CONTROLLER_OS_CTRL_HPP */

#endif /* CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP */
