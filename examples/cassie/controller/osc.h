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
#include "model/spring_deflection.h"
#include "model/springs.h"

// Sizes
#define CASSIE_NQ (23)
#define CASSIE_NV (22)
#define CASSIE_NU (10)

namespace sym = damotion::symbolic;
namespace opt = damotion::optimisation;
namespace osc = damotion::control::osc;

class CassieOSC {
   public:
    /**
     * @brief Constructs the OSC program for the cassie-fixed model
     *
     */
    CassieOSC();
    ~CassieOSC() = default;

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
        std::cout << "u: "
                  << solver_->GetVariableValues(osc_->GetVariables().ctrl())
                         .transpose()
                  << std::endl;
        // Check current cost values
        for (auto& c : solver_->GetCurrentProgram().GetAllCostBindings()) {
            std::cout << c.Get().name() << " "
                      << c.Get().ObjectiveFunction().getOutput(0) << std::endl;
        }
    }

    Eigen::VectorXd CurrentControlSolution() {
        Eigen::VectorXd u =
            solver_->GetVariableValues(osc_->GetVariables().ctrl());
        return u;
    }

   protected:
   private:
    // Left foot back
    std::shared_ptr<osc::ContactTask3D> foot_lb_;
    // Left foot front
    std::shared_ptr<osc::ContactTask3D> foot_lf_;
    // Right foot back
    std::shared_ptr<osc::ContactTask3D> foot_rb_;
    // Right foot front
    std::shared_ptr<osc::ContactTask3D> foot_rf_;

    // Pelvis orientation
    std::shared_ptr<osc::OrientationTask> pelvis_;

    // CoM position
    std::shared_ptr<osc::PositionTask> com_;

    // Heel-spring deflection estimator
    std::unique_ptr<SpringDeflectionEstimator> heel_spring_estimator_;

    // OSC program
    std::unique_ptr<osc::OSC> osc_;
    // Solver
    std::unique_ptr<opt::solvers::QPOASESSolverInstance> solver_;

    Eigen::VectorXd qpos_;
    Eigen::VectorXd qvel_;
};

#endif /* CONTROLLER_OSC_FIXED_H */
