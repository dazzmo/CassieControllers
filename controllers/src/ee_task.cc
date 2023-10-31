#include "controllers/tasks/ee_task.h"

EndEffectorTask::EndEffectorTask(int nv, const std::string &name, f_casadi_cg callback) : Task(3, nv, name, callback) {
    inContact = false;
    mu_ = 1.0;
    lambda_.setZero();
    normal_ = Eigen::Vector3d::UnitZ();
}