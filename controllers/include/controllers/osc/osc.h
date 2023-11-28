#ifndef OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP
#define OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP

#include <iostream>
#include <map>
#include <memory>
#include <qpOASES.hpp>
#include <utility>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/QR"

#include "controllers/acc_limits.h"
#include "controllers/controller.h"
#include "controllers/optimisation/qpoases_data.h"
#include "controllers/optimisation/subvector.h"
#include "controllers/osc/model.h"
#include "controllers/osc/options.h"
#include "controllers/osc/tasks/end_effector_task.h"
#include "controllers/osc/tasks/task.h"
#include "controllers/types.h"

namespace controller {
namespace osc {

class OperationalSpaceController : public Controller {
   public:
    OperationalSpaceController(const Model& model, const Options& opt = Options());
    ~OperationalSpaceController() {
        delete x_;
        delete qp_data_;
    }

    void SetControlWeighting(const Vector& W) { 
        assert(W.size() == m_.size().nu && "Weighting diagonal is not correct length");
        Wu_.diagonal() = W; 
        }

    void Init();
    void UpdateControl(Scalar time, const ConfigurationVector& q, const TangentVector& v);

    /**
     * @brief Resets the controller
     *
     */
    void Reset() { hot_start_ = false; }

    Model& GetModel() { return m_; }

   protected:
    // Projected constraint jacobian
    Matrix Jcp_;
    // Projected constraint jacobian derivative velocity product
    Vector dJcpdq_v_;
    // Null space projector for projected constraints
    Matrix N_;

    // Quadratic programming
    qpOASES::Options qp_opt_;
    std::unique_ptr<qpOASES::SQProblem> qp_;

    optimisation::QPOASESData* qp_data_;

   private:
    Model& m_;
    Options& opt_;

    bool is_initialised_ = false;
    bool hot_start_ = false;

    DiagonalMatrix Wu_;  // Control weighting diagonal matrix

    /**
     * @brief Struct containing optimisation vector components
     *
     */
    struct OptimisationVector {
        OptimisationVector(const DynamicModel::Size& sz, Dimension ncontacts, Dimension nconstraints)
            : qacc(0, sz.nv), lambda_c(0, 3 * ncontacts), lambda_h(0, nconstraints), ctrl(0, sz.nu) {
            lambda_c.InsertAfter(qacc);
            lambda_h.InsertAfter(lambda_c);
            ctrl.InsertAfter(lambda_h);

            // Get overall vector size
            this->sz = qacc.sz + lambda_c.sz + lambda_h.sz + ctrl.sz;
        }

        void Extract(const Vector& x) {
            qacc.vec = x.middleRows(qacc.start, qacc.sz);
            if (lambda_c.sz) lambda_c.vec = x.middleRows(lambda_c.start, lambda_c.sz);
            if (lambda_h.sz) lambda_h.vec = x.middleRows(lambda_h.start, lambda_h.sz);
            ctrl.vec = x.middleRows(ctrl.start, ctrl.sz);
        }

        Dimension sz;

        optimisation::SubVector qacc;
        optimisation::SubVector lambda_c;
        optimisation::SubVector lambda_h;
        optimisation::SubVector ctrl;
    };

    /**
     * @brief Struct containing constraint vector components
     *
     */
    struct ConstraintVector {
        ConstraintVector(const DynamicModel::Size& sz, Dimension ncontact, Dimension nconstraint)
            : dynamics(0, sz.nv),
              friction_cones(0, 4 * ncontact),
              constraint_forces(0, nconstraint) {
            friction_cones.InsertAfter(dynamics);
            constraint_forces.InsertAfter(friction_cones);

            // Get overall vector size
            this->sz = dynamics.sz + friction_cones.sz + constraint_forces.sz;
        }

        void Extract(const Vector& x) {
            dynamics.vec = x.middleRows(dynamics.start, dynamics.sz);
            if (friction_cones.sz) friction_cones.vec = x.middleRows(friction_cones.start, friction_cones.sz);
            if (constraint_forces.sz) constraint_forces.vec = x.middleRows(constraint_forces.start, constraint_forces.sz);
        }

        Dimension sz;

        optimisation::SubVector dynamics;
        optimisation::SubVector friction_cones;
        optimisation::SubVector constraint_forces;
    };

    OptimisationVector* x_;
    ConstraintVector* c_;
};

}  // namespace osc
}  // namespace controller

#endif /* OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP */
