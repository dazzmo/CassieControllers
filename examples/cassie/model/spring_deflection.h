#ifndef MODEL_SPRING_DEFLECTION_H
#define MODEL_SPRING_DEFLECTION_H

#include <damotion/utils/eigen_wrapper.h>
#include <damotion/utils/codegen.h>

#include "model/closed_loop_constraint.h"

class SpringDeflectionEstimator {
   public:
    SpringDeflectionEstimator(pinocchio::ModelTpl<casadi::SX> &model,
                              pinocchio::DataTpl<casadi::SX> &data,
                              const casadi::SX &qpos, const casadi::SX &qvel,
                              bool codegen = false);

    bool EstimateHeelSpringDeflection(
        Eigen::Ref<Eigen::VectorXd> &qpos_leg, const int max_it = 50,
        const double eps = 1e-4);

   private:
    // Constraint Jacobian
    damotion::utils::casadi::FunctionWrapper J_;
};

#endif /* MODEL_SPRING_DEFLECTION_H */
