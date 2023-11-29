#ifndef CONTROLLERS_OSC_MODEL_HPP
#define CONTROLLERS_OSC_MODEL_HPP

#include <iostream>
#include <map>
#include <memory>

#include "controllers/constraint.h"
#include "controllers/model.h"
#include "controllers/osc/tasks/task.h"
#include "controllers/osc/tasks/end_effector_task.h"

namespace controller
{
    namespace osc
    {

        /**
         * @brief Model used for OSC. Holds dynamic data, as well as the
         * tasks, constraints and contact tasks for the model.
         *
         */
        class Model : public DynamicModel
        {
        public:
            Model(const DynamicModel::Size &sz);
            ~Model(){};

            void AddHolonomicConstraint(const std::string &name, Dimension n, Constraint::ConstraintCallbackFunction callback);
            void AddProjectedConstraint(const std::string &name, Dimension n, Constraint::ConstraintCallbackFunction callback);

            void AddTask(const std::string &name, const std::shared_ptr<Task> &task);
            void AddTask(const std::string &name, Dimension n, Task::TaskCallbackFunction callback);
            void AddEndEffectorTask(const std::string &name, Task::TaskCallbackFunction callback);

            std::shared_ptr<Constraint> GetHolonomicConstraint(const std::string &name) { return holonomic_constraints_[name]; }
            std::shared_ptr<Constraint> GetProjectedConstraint(const std::string &name) { return projected_constraints_[name]; }

            std::shared_ptr<Task> GetTask(const std::string &name) { return tasks_[name]; }
            std::shared_ptr<EndEffectorTask> GetEndEffectorTask(const std::string &name) { return end_effector_tasks_[name]; }

            std::map<std::string, std::shared_ptr<Constraint>> &GetHolonomicConstraintMap() { return holonomic_constraints_; }
            std::map<std::string, std::shared_ptr<Constraint>> &GetProjectedConstraintMap() { return projected_constraints_; }

            std::map<std::string, std::shared_ptr<Task>> &GetTaskMap() { return tasks_; }
            std::map<std::string, std::shared_ptr<EndEffectorTask>> &GetEndEffectorTaskMap() { return end_effector_tasks_; }

            const Dimension GetNumberOfContacts() const { return ncontacts_; }
            const Dimension GetNumberOfHolonomicConstraints() const { return nholonomic_constraints_; }
            const Dimension GetNumberOfProjectedConstraints() const { return nprojected_constraints_; }

            void SetControlWeighting(const ActuationVector &Wu) { Wu_ = Wu; }
            const ActuationVector &GetControlWeighting() { return Wu_; }

            virtual void UpdateReferences(Scalar time, const ConfigurationVector &q, const TangentVector &v) {}

        private:
            Dimension ntasks_;                 // Number of tasks
            Dimension ncontacts_;              // Number of contact points
            Dimension nholonomic_constraints_; // Number of holonomic constraints
            Dimension nprojected_constraints_; // Number of projected constraints

            ActuationVector Wu_; // Weighting for OSC control regularisation

            std::map<std::string, std::shared_ptr<Task>> tasks_;
            std::map<std::string, std::shared_ptr<EndEffectorTask>> end_effector_tasks_;

            std::map<std::string, std::shared_ptr<Constraint>> holonomic_constraints_;
            std::map<std::string, std::shared_ptr<Constraint>> projected_constraints_;
        };

    } // namespace osc

} // namespace controller

#endif /* CONTROLLERS_OSC_MODEL_HPP */
