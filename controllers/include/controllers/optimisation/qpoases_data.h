#ifndef CONTROLLERS_OPTIMISATION_QPOASES_DATA_HPP
#define CONTROLLERS_OPTIMISATION_QPOASES_DATA_HPP

#include <qpOASES.hpp>

#include "controllers/types.h"

namespace controller {
namespace optimisation {

struct QPOASESData {
    /**
     * @brief Construct a new QPOASESData object
     *
     * @param nx Number of variables in the problem
     * @param nc Number of box-bounded constraints in the problem
     */
    QPOASESData(Dimension nx, Dimension nc) {
        this->nx = nx;
        this->nc = nc;
        H = Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor>::Zero(nx, nx);
        g = Vector::Zero(nx);
        A = Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor>::Zero(nc, nx);
        ubA = qpOASES::INFTY * Vector::Ones(nc);
        lbA = -qpOASES::INFTY * Vector::Ones(nc);
        ubx = qpOASES::INFTY * Vector::Ones(nx);
        lbx = -qpOASES::INFTY * Vector::Ones(nx);
        x = Vector::Zero(nx);
    }
    // Number of variables
    Dimension nx;
    // Number of equality constraints
    Dimension nc;

    // Solution vector
    Vector x;
    // Hessian matrix of objective
    Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> H;
    // Gradient vector of objective
    Vector g;
    // QP constraint jacobian
    Eigen::Matrix<Scalar, -1, -1, Eigen::RowMajor> A;
    // QP constraint lower bound
    Vector lbA;
    // QP constraint upper bound
    Vector ubA;
    // QP variables lower bound
    Vector lbx;
    // QP variables upper bound
    Vector ubx;
    // Constant component of objective
    Scalar cost_const;
};

}  // namespace optimisation
}  // namespace controller

#endif /* CONTROLLERS_OPTIMISATION_QPOASES_DATA_HPP */
