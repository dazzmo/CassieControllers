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
#include "controllers/osc/tasks/joint_limits_task.h"
#include "controllers/osc/tasks/joint_track_task.h"
#include "controllers/osc/tasks/task.h"
#include "controllers/types.h"

namespace controller {
namespace osc {

class OperationalSpaceController : public Controller {
   public:
    OperationalSpaceController(const Model& model);
    ~OperationalSpaceController() {}

    void SetTorqueWeight(double weight) { torque_weight_ = weight; }

    void CreateOSC(const Options& opt = Options());
    const ActuationVector& RunOSC();

    std::shared_ptr<Task> GetTask(const std::string& name) { return tasks_[name]; }
    std::map<std::string, std::shared_ptr<EndEffectorTask>>& GetEndEffectorTaskMap() { return ee_tasks_; }

    const Model& GetModel() { return m_; }

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
         * @param nce Number of equality constraints in the problem
         */
        QPData(Dimension nx, Dimension nce) {
            this->nx = nx;
            this->nce = nce;
            H = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nx, nx);
            g = Vector::Zero(nx);
            A = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nce, nx);
            ubA = qpOASES::INFTY * Vector::Ones(nce);
            lbA = -qpOASES::INFTY * Vector::Ones(nce);
            ubx = qpOASES::INFTY * Vector::Ones(nx);
            lbx = -qpOASES::INFTY * Vector::Ones(nx);
            x = Vector::Zero(nx);
        }
        // Number of variables
        Dimension nx;
        // Number of equality constraints
        Dimension nce;

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

    struct OptimisationVariable {
        OptimisationVariable(Index start, Dimension sz) {
            this->start = start;
            this->sz = sz;
            vec = Vector::Zero(sz);
        }

        OptimisationVariable() {
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
        void InsertAfter(const OptimisationVariable& v) {
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
            if (opt.use_constraint_nullspace_projector) {
                lambda_c.InsertAfter(qacc);
                ctrl.InsertAfter(lambda_c);
                // Clear lambda_h
                lambda_h = OptimisationVariable();
            } else {
                lambda_c.InsertAfter(qacc);
                lambda_h.InsertAfter(lambda_c);
                ctrl.InsertAfter(lambda_h);
            }

            // Get overall vector size
            this->sz = qacc.sz + lambda_c.sz + lambda_h.sz + ctrl.sz;
        }

        Dimension sz;

        OptimisationVariable qacc;
        OptimisationVariable lambda_c;
        OptimisationVariable lambda_h;
        OptimisationVariable ctrl;
    };

    OptimisationVector* x_;

    JointTrackTask* joint_track_task_;
    JointLimitsTask* joint_limits_task_;

    Options* opt_;

    std::map<std::string, std::shared_ptr<Task>> tasks_;
    std::map<std::string, std::shared_ptr<EndEffectorTask>> ee_tasks_;
};

}  // namespace osc
}  // namespace controller

#endif /* OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP */
