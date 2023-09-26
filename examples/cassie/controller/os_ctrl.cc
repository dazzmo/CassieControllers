#include "os_ctrl.h"

// int CassieOSController::SetupEndEffectors() {
//     RegisterEndEffector("foot_l_front",
//                         cassie_foot_l_front);

//     RegisterEndEffector("foot_l_back",
//                         cassie_foot_l_back);

//     RegisterEndEffector("foot_r_front",
//                         cassie_foot_r_front);

//     RegisterEndEffector("foot_r_back",
//                         cassie_foot_r_back);
//     return 0;
// }

// void CassieOSController::UpdateDynamics() {
//     // Compute dynamic matrices and nullspace projectors
//     const double *in[] = {qpos_.data(), qvel_.data()};
//     double *out[2];
//     out[0] = h_.data();
//     cassie_bias_vector(in, out, NULL, NULL, 0);
//     out[0] = M_.data();
//     cassie_mass_matrix(in, out, NULL, NULL, 0);
//     out[0] = B_.data();
//     cassie_actuation_matrix(in, out, NULL, NULL, 0);
//     out[0] = N_.data();
//     out[1] = gamma_.data();
//     cassie_nullspace_projector(in, out, NULL, NULL, 0);

//     // Compute jacobians for constraints
//     dyn_A_qacc_ = N_ * M_;
//     dyn_A_ctrl_ = -N_ * B_;

//     for (const auto &ee : GetEndEffectorTaskMap()) {
//         dyn_A_lambda_.middleCols(ee.second->lambda_idx, 3) = -N_ * ee.second->J.transpose();
//     }

//     dyn_b_ = -N_ * h_ + gamma_;
// }

// int CassieOSController::HeelSpringDeflection() {
//     Eigen::MatrixXd J(2, 2);
//     Eigen::VectorXd qj = qpos().bottomRows(16);
//     Eigen::VectorXd e(2);
//     Eigen::VectorXd hs(2);

//     const double *in[] = {qj.data()};
//     double *out[2];
//     out[0] = e.data();
//     out[1] = J.data();

//     std::cout << "qj: " << qj.transpose() << std::endl;

//     const int max_iter = 50;
//     for (int i = 0; i < max_iter; i++) {
//         // Perform least-squares estimation
//         cassie_heel_spring_estimation(in, out, NULL, NULL, 0);

//         // Update
//         hs -= (J.transpose() * J).inverse() * J.transpose() * e;
//         std::cout << "hs: " << hs.transpose() << std::endl;
//         std::cout << "e: " << e.transpose() << std::endl;

//         // Update heel spring estimates in joint vector
//         qj[6] = hs[0];
//         qj[14] = hs[1];

//         if (e.lpNorm<Eigen::Infinity>() < 1e-4) {
//             return 0;
//         }
//     }

//     return 1;
// }
