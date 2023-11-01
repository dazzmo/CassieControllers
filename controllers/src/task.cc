#include "controllers/tasks/task.h"

Task::Task(int dim, int nv, const std::string &name) {
    dim_ = dim;
    nv_ = nv;
    name_ = name;
    callback_ = nullptr;

    w_ = 1.0;

    Kp_ = Eigen::VectorXd::Ones(dim);
    Kd_ = Eigen::VectorXd::Ones(dim);

    Resize(dim_, nv_);
}

Task::Task(int dim, int nv, const std::string &name, int (*callback)(const double **, double **)) {
    dim_ = dim;
    nv_ = nv;
    name_ = name;
    callback_ = callback;

    Kp_ = Eigen::VectorXd::Ones(dim);
    Kd_ = Eigen::VectorXd::Ones(dim);

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
    dJdq_ = Eigen::VectorXd::Zero(dim);

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
        out[2] = dJdq_.data();
        // Use callback
        callback_(in, out);
        // Compute task velocity
        dx_ = J_ * qvel;

    } else {
        out[1] = nullptr;
        out[2] = nullptr;
        callback_(in, out);
    }

    return 0;
}

void Task::PrintTaskData() {
    LOG(INFO) << "Task: " << name_ << '\n'
              << "x: " << x_.transpose() << '\n'
              << "r: " << r_.transpose() << '\n'
              << "dx: " << dx_.transpose() << '\n'
              << "dr: " << dr_.transpose() << '\n'
              << "e: " << TaskError().transpose() << '\n'
              << "de: " << TaskErrorDerivative().transpose() << '\n'
              << "e (PD): " << TaskErrorPD().transpose() << '\n'
              << "J: " << J_ << '\n'
              << "dJdq: " << dJdq_ << '\n';
}
