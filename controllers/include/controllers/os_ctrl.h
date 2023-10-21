#ifndef OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP
#define OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP

#include <iostream>
#include <map>
#include <memory>
#include <qpOASES.hpp>
#include <utility>

#include "controllers/controller.h"
#include "controllers/tasks/acc_limits.h"
#include "controllers/tasks/ee_task.h"
#include "controllers/tasks/task.h"

class OperationalSpaceController : public Controller {
   public:
    OperationalSpaceController();
    ~OperationalSpaceController() = default;

    int RegisterTask(const char* name, f_casadi_cg callback);
    int RegisterEndEffector(const char* name, f_casadi_cg callback);

    int UpdateEndEffectorTasks();

    int SetContact(const char* name, double mu, const Eigen::Vector3d& normal);
    int RemoveContact(const char* name);

    int SetupOSC();
    const Eigen::VectorXd& RunOSC();

    std::shared_ptr<Task> GetTask(const std::string& name) { return tasks_[name]; }
    std::map<std::string, std::shared_ptr<EndEffectorTask>>& GetEndEffectorTaskMap() { return ee_tasks_; }

   protected:
    // Number of contact sites
    int nc_;

    // Dynamics
    Eigen::VectorXd dyn_b_;  // Vector for dynamic constraints

    // Quadratic programming
    qpOASES::Options qp_opt_;
    std::unique_ptr<qpOASES::SQProblem> qp_;

    // Solution vector
    Eigen::VectorXd x_;

    // QP Hessian matrix
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> H_;
    // QP gradient vector
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> g_;
    // QP constraint jacobian
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> A_;
    // QP constraint lower bound
    Eigen::VectorXd lbA_;
    // QP constraint upper bound
    Eigen::VectorXd ubA_;
    // QP variables lower bound
    Eigen::VectorXd lbx_;
    // QP variables upper bound
    Eigen::VectorXd ubx_;

    /**
     * @brief Jacobian for the dynamics of the system with respect to qacc, a sub-matrix in the
     * constraint jacobian for the QP.
     *
     * @return Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
     */
    Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> DynamicsQaccJacobian() { return A_.topRows(nv_).middleCols(0, nv_); }
    
    /**
     * @brief Jacobian for the dynamics of the system with respect to lambda, a sub-matrix in the
     * constraint jacobian for the QP.
     *
     * @return Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
     */
    Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> DynamicsLambdaJacobian() { return A_.topRows(nv_).middleCols(nv_, 3 * nc_); }
    
    /**
     * @brief Jacobian for the dynamics of the system with respect to u, a sub-matrix in the
     * constraint jacobian for the QP.
     *
     * @return Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>
     */
    Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> DynamicsCtrlJacobian() { return A_.topRows(nv_).middleCols(nv_ + 3 * nc_, nu_); }
    
    Eigen::Ref<Eigen::VectorXd> DynamicsConstraintVector() { return dyn_b_; }

   private:
    std::map<std::string, std::shared_ptr<Task>> tasks_;
    std::map<std::string, std::shared_ptr<EndEffectorTask>> ee_tasks_;

    // Variables
};

#endif /* OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP */
