#include "controllers/ee_task.h"

EndEffectorTask::EndEffectorTask(int nv, f_casadi_cg callback) : Task(3, nv, callback) {
    lambda_.setZero();
    normal_ = Eigen::Vector3d::UnitZ();
}