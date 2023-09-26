#include "controllers/task.h"

Task::Task(int dim, int nv, f_casadi_cg callback) {
    dim_ = dim;
    nv_ = nv;
    callback_ = callback;

    Resize(dim_, nv_);
}

int Task::Resize(int dim, int nv) {
    x_ = Eigen::VectorXd::Zero(dim);
    dx_ = Eigen::VectorXd::Zero(dim);
    ddx_ = Eigen::VectorXd::Zero(dim);

    r_ = Eigen::VectorXd::Zero(dim);
    dr_ = Eigen::VectorXd::Zero(dim);
    ddr_ = Eigen::VectorXd::Zero(dim);

    J_ = Eigen::MatrixXd::Zero(dim, nv);
    dJdv_ = Eigen::VectorXd::Zero(dim);

    return 0;
}

void Task::SetReference(const Eigen::VectorXd &r) {
    r_ = r;
    dr_.setZero();
    ddr_.setZero();
}

void Task::SetReference(const Eigen::VectorXd &r, const Eigen::VectorXd &dr) {
    r_ = r;
    dr_ = dr;
    ddr_.setZero();
}

void Task::SetReference(const Eigen::VectorXd &r, const Eigen::VectorXd &dr, const Eigen::VectorXd &ddr) {
    r_ = r;
    dr_ = dr;
    ddr_ = ddr;
}

int Task::UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians) {
    const double *in[] = {qpos.data(), qvel.data()};
    double *out[3];
    out[0] = x_.data();
    if (update_jacobians) {
        out[1] = J_.data();
        out[2] = dJdv_.data();
    } else {
        out[1] = nullptr;
        out[2] = nullptr;
    }

    callback_(in, out, NULL, NULL, 0);

    return 0;
}
