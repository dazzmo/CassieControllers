#ifndef OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP
#define OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP

#include <iostream>
#include <map>
#include <memory>
#include <qpOASES.hpp>
#include <utility>

#include "controllers/acc_limits.h"
#include "controllers/controller.h"
#include "controllers/osc/model.h"
#include "controllers/osc/options.h"
#include "controllers/osc/tasks/ee_task.h"
#include "controllers/osc/tasks/task.h"
#include "controllers/types.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/QR"

namespace controller {
namespace osc {

class OperationalSpaceController : public Controller {
   public:
    OperationalSpaceController(const Model& model);
    ~OperationalSpaceController() {
        delete x_;
        delete qp_data_;
        delete opt_;
    }

    void SetTorqueWeight(double weight) { torque_weight_ = weight; }

    void CreateOSC(const Options& opt = Options());
    void UpdateControl(Scalar time, const ConfigurationVector& q, const TangentVector& v);

    Model& GetModel() { return m_; }

   protected:
    // Null space projector for holonomic constraints
    Matrix N_;
    // Holonomic constraint jacobian
    Matrix Jceq_;
    // Holonomic constraint jacobian derivative velocity product
    Vector dJceqdq_;

    // Quadratic programming
    qpOASES::Options qp_opt_;
    std::unique_ptr<qpOASES::SQProblem> qp_;

    struct QPData {
        /**
         * @brief Construct a new QPData object
         *
         * @param nx Number of variables in the problem
         * @param nc Number of box-bounded constraints in the problem
         */
        QPData(Dimension nx, Dimension nc) {
            this->nx = nx;
            this->nc = nc;
            H = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nx, nx);
            g = Vector::Zero(nx);
            A = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nc, nx);
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
        // QP Hessian matrix
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> H;
        // QP gradient vector
        Vector g;
        // QP constraint jacobian
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> A;
        // QP constraint lower bound
        Vector lbA;
        // QP constraint upper bound
        Vector ubA;
        // QP variables lower bound
        Vector lbx;
        // QP variables upper bound
        Vector ubx;
    };

    QPData* qp_data_;

   private:
    Model& m_;

    bool osc_setup_ = false;
    bool hot_start_ = false;

    double torque_weight_ = 1.0;

    struct SubVector {
        SubVector(Index start, Dimension sz) {
            this->start = start;
            this->sz = sz;
            vec = Vector::Zero(sz);
        }

        SubVector() {
            this->start = 0;
            this->sz = 0;
            vec = Vector::Zero(0);
        }

        /**
         * @brief Inserts this variable directly after v in the optimisation
         * vector
         *
         * @param v
         */
        void InsertAfter(const SubVector& v) {
            this->start = v.start + v.sz;
        }

        Index start;
        Dimension sz;
        Vector vec;
    };

    struct OptimisationVector {
        OptimisationVector(const DynamicModel::Size& sz, Dimension nc, Dimension nceq, const Options& opt)
            : qacc(0, sz.nv), lambda_c(0, 3 * nc), lambda_h(0, nceq), ctrl(0, sz.nu) {
            // Add constraint forces explicitly or implicitly
            if (opt.include_constraint_forces) {
                lambda_c.InsertAfter(qacc);
                lambda_h.InsertAfter(lambda_c);
                ctrl.InsertAfter(lambda_h);
            } else {
                lambda_c.InsertAfter(qacc);
                ctrl.InsertAfter(lambda_c);
                // Clear lambda_h
                lambda_h = SubVector();
            }

            // Get overall vector size
            this->sz = qacc.sz + lambda_c.sz + lambda_h.sz + ctrl.sz;
        }

        Dimension sz;

        SubVector qacc;
        SubVector lambda_c;
        SubVector lambda_h;
        SubVector ctrl;
    };

    struct ConstraintVector {
        ConstraintVector(const DynamicModel::Size& sz, Dimension nc, Dimension nceq, const Options& opt)
            : dyn(0, sz.nu), lambda_c(0, 3 * nc), lambda_h(0, nceq) {
            lambda_c.InsertAfter(dyn);
            // Add constraint forces explicitly or implicitly
            if (opt.include_constraint_forces) {
                lambda_h.InsertAfter(lambda_c);
            } else {
                lambda_h = SubVector();
            }

            // Get overall vector size
            this->sz = dyn.sz + lambda_c.sz + lambda_h.sz;
        }

        Dimension sz;

        SubVector dyn;
        SubVector lambda_c;
        SubVector lambda_h;
    };

    OptimisationVector* x_;
    ConstraintVector* c_;
    Options* opt_;
};

}  // namespace osc
}  // namespace controller

#endif /* OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP */
