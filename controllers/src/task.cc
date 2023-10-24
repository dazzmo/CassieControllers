#include "controllers/tasks/task.h"

Task::Task(int dim, int nv) {
    dim_ = dim;
    nv_ = nv;
    callback_ = nullptr;

    Kp_ = Eigen::VectorXd::Ones(dim);
    Kd_ = Eigen::VectorXd::Ones(dim);

    Resize(dim_, nv_);
}

Task::Task(int dim, int nv, f_casadi_cg callback) {
    dim_ = dim;
    nv_ = nv;
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

    w_ = Eigen::VectorXd::Zero(dim);

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
        callback_(in, out, NULL, NULL, 0);
        // Compute task velocity
        dx_ = J_ * qvel;

        LOG(INFO) << "J: " << J_;
        LOG(INFO) << "dJdq: " << dJdq_.transpose();
        LOG(INFO) << "x: " << x_.transpose();
        LOG(INFO) << "dx: " << dx_.transpose();

    } else {
        out[1] = nullptr;
        out[2] = nullptr;
        callback_(in, out, NULL, NULL, 0);
    }

    return 0;
}

void Task::PrintTaskData() {
    LOG(INFO) << "Task: ";
    LOG(INFO) << "x: " << x_.transpose();
    LOG(INFO) << "r: " << r_.transpose();
    LOG(INFO) << "dx: " << dx_.transpose();
    LOG(INFO) << "dr: " << dr_.transpose();
    LOG(INFO) << "e: " << TaskError().transpose();
    LOG(INFO) << "de: " << TaskErrorDerivative().transpose();
    LOG(INFO) << "e (PD): " << TaskErrorPD().transpose();
}
