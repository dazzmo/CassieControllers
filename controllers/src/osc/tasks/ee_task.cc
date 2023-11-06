#include "controllers/osc/tasks/ee_task.h"

using namespace controller::osc;

EndEffectorTask::EndEffectorTask(const std::string& name, const DynamicModel::Size& sz, Task::TaskCallbackFunction& callback) : Task(name, 3, sz, callback) {
    inContact = false;
    mu_ = 1.0;
    lambda_.setZero();
    normal_ = Eigen::Vector3d::UnitZ();
}