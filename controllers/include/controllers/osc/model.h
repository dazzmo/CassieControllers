#ifndef CONTROLLERS_OSC_MODEL_HPP
#define CONTROLLERS_OSC_MODEL_HPP

#include <iostream>
#include <map>
#include <memory>

#include "controllers/constraint.h"
#include "controllers/model.h"
#include "controllers/osc/tasks/ee_task.h"
#include "controllers/osc/tasks/task.h"

namespace controller {
namespace osc {

/**
 * @brief Model used for OSC. Holds dynamic data, as well as the
 * tasks, constraints and contact tasks for the model.
 *
 */
class Model : public DynamicModel {
   public:
    Model(const DynamicModel::Size &sz);
    ~Model(){};

    void AddConstraint(const std::string &name, Dimension n, Constraint::ConstraintCallbackFunction callback);
    void AddTask(const std::string &name, Dimension n, Task::TaskCallbackFunction callback);
    void AddEndEffectorTask(const std::string &name, Task::TaskCallbackFunction callback);

    std::shared_ptr<Constraint> GetConstraint(const std::string &name) { return constraints_[name]; }
    std::shared_ptr<Task> GetTask(const std::string &name) { return tasks_[name]; }
    std::shared_ptr<EndEffectorTask> GetEndEffectorTask(const std::string &name) { return end_effector_tasks_[name]; }

    std::map<std::string, std::shared_ptr<Constraint>> &GetConstraintMap() { return constraints_; }
    std::map<std::string, std::shared_ptr<Task>> &GetTaskMap() { return tasks_; }
    std::map<std::string, std::shared_ptr<EndEffectorTask>> &GetEndEffectorTaskMap() { return end_effector_tasks_; }

    const Dimension GetNumberOfContacts() const { return ncontacts_; }
    const Dimension GetNumberOfConstraints() const { return nconstraints_; }

    virtual void UpdateReferences(Scalar time, const ConfigurationVector &q, const TangentVector &v) {}

   private:
    // Number of tasks
    Dimension ntasks_;
    // Number of contact points
    Dimension ncontacts_;
    // Number of equality constraints
    Dimension nconstraints_;

    std::map<std::string, std::shared_ptr<Task>> tasks_;
    std::map<std::string, std::shared_ptr<EndEffectorTask>> end_effector_tasks_;
    std::map<std::string, std::shared_ptr<Constraint>> constraints_;
};

}  // namespace osc

}  // namespace controller

#endif /* CONTROLLERS_OSC_MODEL_HPP */
