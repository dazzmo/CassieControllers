#include "controllers/osc/tasks/task.h"
using namespace controller::osc;

Task::Task(const std::string &name, Dimension n, const DynamicModel::Size &sz) {
    name_ = name;
    callback_ = nullptr;
    Resize(n, sz);
}

Task::Task(const std::string &name, Dimension n, const DynamicModel::Size &sz, TaskCallbackFunction callback) {
    name_ = name;
    callback_ = callback;
    Resize(n, sz);
}

void Task::Resize(Dimension n, const DynamicModel::Size &sz) {
    nq_ = sz.nq;
    nv_ = sz.nv;

    w_ = 1.0;

    Kp_ = Vector::Ones(n);
    Kd_ = Vector::Ones(n);

    // Create vectors
    x_ = Vector::Zero(n);
    dx_ = Vector::Zero(n);
    ddx_ = Vector::Zero(n);

    r_ = Vector::Zero(n);
    dr_ = Vector::Zero(n);
    ddr_ = Vector::Zero(n);

    e_ = Vector::Zero(n);
    de_ = Vector::Zero(n);
    dde_ = Vector::Zero(n);

    pd_out_ = Vector::Zero(n);

    J_ = Matrix::Zero(n, sz.nv);
    dJdq_ = Vector::Zero(n);
}

void Task::SetReference(const Vector &r) {
    r_ = r;
    dr_.setZero();
    ddr_.setZero();
}

void Task::SetReference(const Vector &r, const Vector &dr) {
    r_ = r;
    dr_ = dr;
    ddr_.setZero();
}

void Task::SetReference(const Vector &r, const Vector &dr, const Vector &ddr) {
    r_ = r;
    dr_ = dr;
    ddr_ = ddr;
}

void Task::Update(const ConfigurationVector &q, const TangentVector &v) {
    // Perform callback
    if (callback_ != nullptr) {
        callback_(q, v, x_, J_, dJdq_);
    } else {
        throw std::runtime_error("Task callback is null");
    }

    // Update velocity
    dx_ = J_ * v;
    // TODO: Update acceleration also (need measurements through estimation)
    // ddx_ = J_ * a + dJdq_;

    // Compute errors
    e_ = r_ - x_;
    de_ = dr_ - dx_;
    // TODO: Accelerations
    // dde_ = ddr_ - ddx_;

    // Compute PD error
    pd_out_ = Kp_.asDiagonal() * e_ + Kd_.asDiagonal() * de_;
}
