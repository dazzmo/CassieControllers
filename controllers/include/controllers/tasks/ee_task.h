#ifndef INCLUDE_CONTROLLERS_EE_TASK_HPP
#define INCLUDE_CONTROLLERS_EE_TASK_HPP

#include "controllers/tasks/task.h"

class EndEffectorTask : public Task {
   public:

    bool inContact = false;

    EndEffectorTask(int nv, f_casadi_cg callback);
    ~EndEffectorTask() = default;

    void SetId(const int& id) { id_ = id; }
    const int& GetId() const { return id_; }

    void SetFrictionCoefficient(double mu) { mu_ = mu; }
    const double& mu() const { return mu_; }

    const Eigen::Vector3d& lambda() const { return lambda_; }
    Eigen::Vector3d& lambda() { return lambda_; }

    const Eigen::Vector3d& normal() const { return normal_; }
    Eigen::Vector3d& normal() { return normal_; }

   protected:
   private:
    int id_;
    double mu_;
    Eigen::Vector3d lambda_;
    Eigen::Vector3d normal_;
};

#endif /* INCLUDE_CONTROLLERS_EE_TASK_HPP */
