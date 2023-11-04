#ifndef INCLUDE_CONTROLLERS_EE_TASK_HPP
#define INCLUDE_CONTROLLERS_EE_TASK_HPP

#include "controllers/osc/tasks/task.h"

class EndEffectorTask : public Task {
   public:
    typedef Eigen::Vector3d Vector3d;

    bool inContact = false;

    EndEffectorTask(int nv, const std::string& name, int (*callback)(const double**, double**));
    ~EndEffectorTask() = default;

    void SetId(const int& id) { id_ = id; }
    const int& GetId() const { return id_; }

    void SetFrictionCoefficient(double mu) { mu_ = mu; }
    const double& mu() const { return mu_; }

    Vector3d& lambda() { return lambda_; }
    const Vector3d& lambda() const { return lambda_; }

    Vector3d& normal() { return normal_; }
    const Vector3d& normal() const { return normal_; }

   protected:
   private:
    int id_;
    double mu_;
    Vector3d lambda_;
    Vector3d normal_;
};

#endif /* INCLUDE_CONTROLLERS_EE_TASK_HPP */
