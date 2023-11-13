#include "controllers/osc/model.h"

using namespace controller::osc;

Model::Model(const DynamicModel::Size &sz) : DynamicModel(sz) {
    ntasks_ = 0;
    ncontacts_ = 0;
    nholonomic_constraints_ = 0;
    nprojected_constraints_ = 0;
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
    tasks_[name]->SetStartIndex(ntasks_);
    ntasks_ += n;
}

/**
 * @brief Adds a new task
 * 
 * @param name 
 * @param task 
 */
void Model::AddTask(const std::string &name, const std::shared_ptr<Task> &task) {
    tasks_[name] = task;
    // Add position to task vector
    tasks_[name]->SetStartIndex(ntasks_);
    ntasks_ += task->dim();
}

/**
 * @brief Adds a three-dimensional end-effector task, where callback is a function with inputs
 * (q, v) and outputs the task, its jacobian and its time derivative-velocity product (i.e. (x, J, dJdq_v))
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
 * @brief Adds a holonomic constraint to the model where the constriant forces associated with
 * the constraint are solved for explicitly within the optimisation.
 *
 * @param name
 * @param n
 * @param callback
 */
void Model::AddHolonomicConstraint(const std::string &name, Dimension n, Constraint::ConstraintCallbackFunction callback) {
    // Add task to map
    holonomic_constraints_[name] = std::shared_ptr<Constraint>(new Constraint(name, n, this->size(), callback));
    // Add position to constraint vector
    holonomic_constraints_[name]->SetStartIndex(nholonomic_constraints_);
    // Increase number of constraints problem
    nholonomic_constraints_ += n;
}

/**
 * @brief Adds a constraint to the model that the dynamic constriants will be projected into
 * (i.e. constraint forces are anaytically determined before optimisation)
 *
 * @param name
 * @param n
 * @param callback
 */
void Model::AddProjectedConstraint(const std::string &name, Dimension n, Constraint::ConstraintCallbackFunction callback) {
    // Add task to map
    projected_constraints_[name] = std::shared_ptr<Constraint>(new Constraint(name, n, this->size(), callback));
    // Add position to constraint vector
    projected_constraints_[name]->SetStartIndex(nprojected_constraints_);
    // Increase number of constraints problem
    nprojected_constraints_ += n;
}