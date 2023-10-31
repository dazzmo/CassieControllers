#ifndef CASSIE_CONTROLLER_OS_CTRL_HPP
#define CASSIE_CONTROLLER_OS_CTRL_HPP

#include <glog/logging.h>

#include "controllers/os_ctrl.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Cholesky"
#include "model/cg/arm_actuation_matrix.h"
#include "model/cg/arm_bias_vector.h"
#include "model/cg/arm_mass_matrix.h"
#include "model/cg/arm_constraint.h"
#include "model/cg/arm_tip.h"

#define ARM_MODEL_NQ 3 
#define ARM_MODEL_NV 3 
#define ARM_MODEL_NU 3 

class ArmOSC : public OperationalSpaceController {
   public:
    ArmOSC() : OperationalSpaceController(ARM_MODEL_NQ, ARM_MODEL_NV, ARM_MODEL_NU) {
        LOG(INFO) << "ArmOSC::ArmOSC()";
    }
    ~ArmOSC() = default;

    void SetupController();
    int UpdateControl();
    void UpdateDynamics();

    int MapMujocoState(const double* q, const double* v);

   protected:
   private:
    Eigen::MatrixXd J_;
    Eigen::MatrixXd M_;
    Eigen::VectorXd h_;
    Eigen::MatrixXd B_;

    int InitMatrices();
};

#endif /* CASSIE_CONTROLLER_OS_CTRL_20COPY_HPP */
