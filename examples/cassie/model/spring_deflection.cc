#include "model/spring_deflection.h"

SpringDeflectionEstimator::SpringDeflectionEstimator(
    pinocchio::ModelTpl<casadi::SX> &model,
    pinocchio::DataTpl<casadi::SX> &data, const casadi::SX &qpos,
    const casadi::SX &qvel, bool codegen) {
    // Compute the constraint for the closed-loop constraints
    casadi::SX cl = CassieClosedLoopConstraint(model, data, qpos, qvel);

    // Get first entry of the constraint (it's the left-leg closed-loop
    // condition, by symmetry this will also apply to the right leg)
    casadi::SX c = cl(0);
    // Set the floating base coordinates to zero
    if (qpos.size1() > 16) {
        c = casadi::SX::substitute(c, qpos(casadi::Slice(0, 7)), 0);
    }

    // Compute the gradient of the constraint with respect to the left
    // heel-spring deflection
    casadi::SX q_leg_l = qpos(casadi::Slice(7, 15));
    casadi::SX g = gradient(c, q_leg_l(6));

    // Create function and generate the expression
    casadi::Function f("heel_spring_estimator", {q_leg_l}, {c, g});
    if (codegen) {
        f = damotion::utils::casadi::codegen(f, "./");
    }
    // Wrap the function
    J_ = damotion::utils::casadi::FunctionWrapper(f);
}

bool SpringDeflectionEstimator::EstimateHeelSpringDeflection(
    Eigen::Ref<Eigen::VectorXd> &qpos_leg, const int max_it, const double eps) {
    Eigen::VectorXd qi = qpos_leg;
    double q_heel = 0.0;

    // Perform Gauss-Newton descent for heel-spring deflection || h(q, q_heel)
    // ||^2 in q_heel to minimise the error by changing q_heel
    for (int i = 0; i < max_it; i++) {
        J_.setInput(0, qi);
        J_.call();
        // Constraint error
        Eigen::VectorXd e = J_.getOutput(0);
        // Constraint Jacobian
        Eigen::MatrixXd Ji = J_.getOutput(1);
        // Perform step given the gradient under Gauss-Newton descent
        q_heel -= ((Ji.transpose() * Ji).inverse() * Ji.transpose() * e)[0];
        // Update heel spring estimate in joint vector
        qi[6] = q_heel;

        if (e.lpNorm<Eigen::Infinity>() < eps) {
            qpos_leg = qi;
            return 0;
        }
    }

    return 1;
}