#ifndef CONTROLLER_OSC_FIXED_H
#define CONTROLLER_OSC_FIXED_H

#include <glog/logging.h>

// pinocchio
#include <pinocchio/parsers/urdf.hpp>

// damotion
#include <damotion/control/osc/osc.h>
#include <damotion/solvers/solve_qpoases.h>

#include "damotion/solvers/bounds.h"
#include "damotion/solvers/program.h"
// model
#include "model/actuation.h"
#include "model/closed_loop_constraint.h"
#include "model/springs.h"

// Sizes
#define CASSIE_FIXED_NQ (16)
#define CASSIE_FIXED_NV (16)
#define CASSIE_FIXED_NU (10)

namespace sym = damotion::symbolic;
namespace opt = damotion::optimisation;
namespace osc = damotion::control::osc;

class CassieFixedOSC {
   public:
    /**
     * @brief Constructs the OSC program for the cassie-fixed model
     *
     */
    CassieFixedOSC();
    ~CassieFixedOSC() = default;

    // Update the references for any tasks
    void UpdateReferences(double time);

    /**
     * @brief Remaps the measured state to the state expected by our model
     *
     * @param nq
     * @param q
     * @param nv
     * @param v
     */
    void UpdateState(int nq, const double* q, int nv, const double* v);

    void Solve() {
        // Update program with current state of system
        osc_->Update(qpos_, qvel_);
        // Update solver with current program
        solver_->UpdateProgram(osc_->GetProgram());

        solver_->Solve();
        // Check current cost values
        for (auto& c : solver_->GetCurrentProgram().GetAllCostBindings()) {
            std::cout << c.Get().name() << " "
                      << c.Get().ObjectiveFunction().getOutput(0) << std::endl;
        }
    }

    Eigen::VectorXd CurrentControlSolution() {
        // TODO - Provide output PD controller for joint stabilisation? Instead
        // TODO - of including it in program?
        return solver_->GetVariableValues(osc_->GetVariables().ctrl());
    }

   protected:
   private:
    // End-effector tasks
    std::shared_ptr<osc::PositionTask> left_foot_;
    std::shared_ptr<osc::PositionTask> right_foot_;

    std::shared_ptr<osc::OrientationTask> right_foot_orientation_;

    // OSC program
    std::unique_ptr<osc::OSC> osc_;
    // Solver
    std::unique_ptr<opt::solvers::QPOASESSolverInstance> solver_;

    Eigen::VectorXd qpos_;
    Eigen::VectorXd qvel_;
};

#endif /* CONTROLLER_OSC_FIXED_H */
