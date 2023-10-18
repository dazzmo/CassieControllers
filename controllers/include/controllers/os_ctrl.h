#ifndef OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP
#define OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP

#include <iostream>
#include <map>
#include <memory>
#include <qpOASES.hpp>
#include <utility>

#include "controllers/controller.h"
#include "controllers/task.h"
#include "controllers/ee_task.h"
#include "controllers/acc_limits.h"

class OperationalSpaceController : public Controller {
   public:
    OperationalSpaceController();
    ~OperationalSpaceController() = default;

    int RegisterTask(const char* name, f_casadi_cg callback);
    int RegisterEndEffector(const char* name, f_casadi_cg callback);

    int UpdateEndEffectorTasks();

    int SetContact(const char* name, double mu, const Eigen::Vector3d& normal);
    int RemoveContact(const char* name);

    int AddQposReference(const Eigen::VectorXd &qpos);
    // int AddQvelReference(const Eigen::VectorXd &qvel);


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

    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> W_;
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> H_;
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> g_;
    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> A_;
    Eigen::VectorXd lbA_;
    Eigen::VectorXd ubA_;
    Eigen::VectorXd lbx_;
    Eigen::VectorXd ubx_;

    Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> DynamicsQaccJacobian() { return A_.topRows(nv_).middleCols(0, nv_); }
    Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> DynamicsLambdaJacobian() { return A_.topRows(nv_).middleCols(nv_, 3 * nc_); }
    Eigen::Ref<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> DynamicsCtrlJacobian() { return A_.topRows(nv_).middleCols(nv_ + 3 * nc_, nu_); }
    Eigen::Ref<Eigen::VectorXd> DynamicsConstraintVector() { return dyn_b_; }

   private:
    std::map<std::string, std::shared_ptr<Task>> tasks_;
    std::map<std::string, std::shared_ptr<EndEffectorTask>> ee_tasks_;

    // Variables
};

#endif /* OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP */
