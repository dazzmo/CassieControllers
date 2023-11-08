#include "controllers/osc/model.h"

using namespace controller::osc;

Model::Model(const DynamicModel::Size &sz) : DynamicModel(sz) {
    ntasks_ = 0;
    ncontacts_ = 0;
    nconstraints_ = 0;
}

/**
 * @brief Adds a new task
 *
 * @param name
 * @param callback
 * @return int
 */
void Model::AddTask(const std::string &name, Dimension n, Task::TaskCallbackFunction callback) {
    tasks_[name] = std::shared_ptr<Task>(new Task(name, n, this->size(), callback));
    // Add position to task vector
    tasks_[name]->SetStartIndex(nt_);
    ntasks_ += n;
}

/**
 * @brief Adds a three-dimensional end-effector task, where callback is a function with inputs
 * (q, v) and outputs the task, its jacobian and its time derivative-velocity product (i.e. (x, J, dJdq))
 *
 * @param name
 * @param callback
 * @return int
 */
void Model::AddEndEffectorTask(const std::string &name, Task::TaskCallbackFunction callback) {
    // Add task to map
    end_effector_tasks_[name] = std::shared_ptr<EndEffectorTask>(new EndEffectorTask(name, this->size(), callback));
    // Add position to end-effector task vector
    tasks_[name]->SetStartIndex(3 * ncontacts_);
    // Increase number of contact points
    ncontacts_ += 1;
}

/**
 * @brief Adds a holonomic constraint to the model
 *
 * @param name
 * @param n
 * @param callback
 */
void Model::AddConstraint(const std::string &name, Dimension n, Constraint::ConstraintCallbackFunction callback) {
    // Add task to map
    constraints_[name] = std::shared_ptr<Constraint>(new Constraint(name, n, this->size(), callback));
    // Add position to constraint vector
    constraints_[name]->SetStartIndex(nceq_);
    // Increase number of constraints problem
    nconstraints_ += n;
}