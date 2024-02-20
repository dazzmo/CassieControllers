#include "controllers/osc/tasks/end_effector_task.h"

using namespace controller::osc;

EndEffectorTask::EndEffectorTask(const std::string& name, const DynamicModel::Size& sz, Task::TaskCallbackFunction& callback) : Task(name, 3, sz, callback) {
    inContact = false;
    mu_ = 1.0; // TODO: Make this a default setting
    lambda_.setZero();
    normal_ = Eigen::Vector3d::UnitZ();
}