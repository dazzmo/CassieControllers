#include "osc_leg.h"

/**
 * @brief Remaps the measured state to the state expected by our model
 * 
 * @param nq 
 * @param q 
 * @param nv 
 * @param v 
 */
void CassieLegOSC::UpdateState(Dimension nq, const Scalar *q, Dimension nv, const Scalar *v) {
    state().q << q[0], q[1], q[2], q[7], q[8], q[9], q[10], q[13];
    state().v << v[0], v[1], v[2], v[6], v[7], v[8], v[9], v[12];
}

/**
 * @brief Estimates the angle of the heel-spring based on the configuration of the leg, given no encoder is provided for the
 * heel spring joint.
 *
 * @return int
 */
int CassieLegOSC::HeelSpringDeflection() {
    Eigen::MatrixXd J(1, 1);
    Eigen::VectorXd e(1), dJdt_v(1);
    Eigen::VectorXd hs(1);
    Eigen::VectorXd qj = state().q;

    const double *in[] = {qj.data()};
    double *out[3] = {e.data(), J.data(), dJdt_v.data()};

    std::cout << "qj: " << qj.transpose() << std::endl;

    const int max_iter = 50;
    for (int i = 0; i < max_iter; i++) {
        // Perform least-squares estimation
        cassie_heel_spring_constraint(in, out, NULL, NULL, 0);
        // Update
        hs -= (J.transpose() * J).inverse() * J.transpose() * e;
        // Update heel spring estimates in joint vector
        qj[5] = hs[0];

        if (e.lpNorm<Eigen::Infinity>() < 1e-4) {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief Performs inverse-kinematics with nonlinear equality constraints
 *
 * @param qpos
 * @param x
 * @param q0
 * @return int
 */
int CassieLegOSC::InverseKinematics(Eigen::VectorXd &qpos, const Eigen::Vector3d &x_d, const Eigen::VectorXd &q0) {
    LOG(INFO) << "Starting IK";
    // Solver options
    const int max_it = 50;
    // Iterates
    Eigen::VectorXd q_i(size().nq);
    if (ik_restart_) {
        q_i = initial_state().q;
    } else {
        q_i = q0;
    }

    Eigen::VectorXd lamba_i(1);
    Eigen::Vector3d x_i;
    // Gradients, Jacobians and Hessians
    Eigen::MatrixXd J(3, 8);
    // Lagrangian Hessian
    Eigen::MatrixXd lag_hess(size().nq, size().nq);

    J.setZero();
    // Constrained Gauss-Newton approach for solving quality-constrained least-squares
    Eigen::VectorXd v, x, lam(1);
    Eigen::MatrixXd jac_kkt(size().nq + 1, size().nq + 1);
    Eigen::VectorXd vec_kkt(size().nq + 1);
    jac_kkt.setZero();
    vec_kkt.setZero();

    const double eps = 1e-3;

    Eigen::VectorXd gi(1);
    Eigen::MatrixXd jac_gi(1, size().nq), hess_gi(size().nq, size().nq);
    LOG(INFO) << "qi: " << q_i.transpose();

    const double *in[] = {q_i.data(), nullptr};

    double *out_ankle[3] = {x_i.data(), J.data(), NULL};
    double *out_constraint[4] = {gi.data(), jac_gi.data(), NULL, hess_gi.data()};

    // Identity matrix
    Eigen::MatrixXd I(size().nq, size().nq);
    I.setZero();
    I.diagonal() << 1e-2, 1e-2, 1e-2, 1e-2, 1e1, 1e-2, 1e1, 1e-2;

    lam.setOnes();

    // TODO: Don't rely on spring deflection as much (add penalisation)

    for (int i = 0; i < max_it; i++) {
        LOG(INFO) << "Iteration " << i;
        // Compute end-effector jacobian
        cassie_ankle(in, out_ankle, NULL, NULL, 0);
        LOG(INFO) << "Ankle data computed";
        // Compute constraint jacobian and hessian
        cassie_heel_spring_constraint(in, out_constraint, NULL, NULL, 0);
        LOG(INFO) << "Constraint data computed";

        // Compute FK error
        LOG(INFO) << "xi = " << x_i.transpose();
        Eigen::VectorXd e = x_i - x_d;
        LOG(INFO) << "e = " << e.transpose();

        if (e.squaredNorm() < eps) break;
        // Compute hessian of lagrangian
        lag_hess = (J.transpose() * J + I) - lam[0] * hess_gi;
        // LOG(INFO) << "lag_hess = " << lag_hess;

        // Compute KKT jacobian
        // LOG(INFO) << "jac_kkt = " << jac_kkt;

        jac_kkt.topLeftCorner(size().nq, size().nq) << lag_hess;
        jac_kkt.bottomLeftCorner(1, size().nq) << jac_gi;
        jac_kkt.topRightCorner(size().nq, 1) << jac_gi.transpose();

        // LOG(INFO) << "jac_kkt = " << jac_kkt;

        // Compute KKT vector
        vec_kkt << J.transpose() * e, gi;

        // Compute step
        Eigen::VectorXd dx = jac_kkt.ldlt().solve(-vec_kkt);

        LOG(INFO) << "dx: " << dx.transpose();

        // Take a step
        q_i += dx.topRows(size().nq);
        lam += dx.bottomRows(1);

        // Clamp results
        for (int i = 0; i < size().nq; i++) {
            q_i[i] = std::min(std::max(q_i[i], bounds().qmin[i]), bounds().qmax[i]);
        }
    }

    LOG(INFO) << "q: " << q_i.transpose();
    qpos = q_i;

    return 0;
}