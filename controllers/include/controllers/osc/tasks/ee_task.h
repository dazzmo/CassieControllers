#ifndef INCLUDE_CONTROLLERS_EE_TASK_HPP
#define INCLUDE_CONTROLLERS_EE_TASK_HPP

#include "controllers/osc/tasks/task.h"

namespace controller {
namespace osc {

class EndEffectorTask : public Task {
   public:
    bool inContact = false;

    EndEffectorTask(const std::string& name, const DynamicModel::Size& sz, Task::TaskCallbackFunction& callback);
    ~EndEffectorTask() = default;

    void SetId(const int& id) { id_ = id; }
    const int& GetId() const { return id_; }

    void SetFrictionCoefficient(Scalar mu) { mu_ = mu; }
    const Scalar& mu() const { return mu_; }

    Vector3& lambda() { return lambda_; }
    const Vector3& lambda() const { return lambda_; }

    Vector3& normal() { return normal_; }
    const Vector3& normal() const { return normal_; }

   protected:
   private:
    int id_;
    Scalar mu_;
    Vector3 lambda_;
    Vector3 normal_;
};

}  // namespace osc
}  // namespace controller
#endif /* INCLUDE_CONTROLLERS_EE_TASK_HPP */
