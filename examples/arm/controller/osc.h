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

// Sizes
#define ARM_NQ (3)
#define ARM_NV (3)
#define ARM_NU (3)

namespace sym = damotion::symbolic;
namespace opt = damotion::optimisation;
namespace osc = damotion::control::osc;

class ArmOSC {
   public:
    /**
     * @brief Constructs the OSC program for the cassie-fixed model
     *
     */
    ArmOSC();
    ~ArmOSC() = default;

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
        solver_->UpdateProgram(program_);
        solver_->Solve();
        // Check current cost values
        for (auto& c : solver_->GetCurrentProgram().GetAllCostBindings()) {
            std::cout << c.Get().name() << " "
                      << c.Get().ObjectiveFunction().getOutput(0) << std::endl;
        }

        Eigen::VectorXd a = solver_->GetVariableValues(qacc_);
        Eigen::VectorXd u = solver_->GetVariableValues(ctrl_);

        std::cout << "a: " << a.transpose() << std::endl;
        std::cout << "u: " << u.transpose() << std::endl;
    }

    Eigen::VectorXd CurrentControlSolution() {
        Eigen::VectorXd u = solver_->GetVariableValues(ctrl_);
        return u;
    }

   protected:
   private:
    // Program
    opt::Program program_;
    // Solver
    std::unique_ptr<opt::solvers::QPOASESSolverInstance> solver_;

    Eigen::VectorXd qpos_;
    Eigen::VectorXd qvel_;

    std::vector<std::shared_ptr<osc::EndEffector>> ee_;

    // Standard optimisation variables
    sym::VariableVector qacc_;
    sym::VariableVector ctrl_;
    // Vector of constraint forces
    std::vector<sym::VariableVector> constraint_forces_;

    // Tasks
    std::unordered_map<std::string, std::unique_ptr<osc::TrackingTaskData>>
        tracking_tasks_;
    std::vector<osc::HolonomicConstraint> constraints_;

};

#endif/* CONTROLLER_OSC_FIXED_H */
