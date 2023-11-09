#include "controllers/constraint.h"

using namespace controller;

Constraint::Constraint(const std::string &name, Dimension n, const DynamicModel::Size &sz) {
    name_ = name;
    callback_ = nullptr;
    Resize(n, sz);
}

Constraint::Constraint(const std::string &name, Dimension n, const DynamicModel::Size &sz, ConstraintCallbackFunction callback) {
    name_ = name;
    callback_ = callback;
    Resize(n, sz);
}

void Constraint::Resize(Dimension n, const DynamicModel::Size &sz) {
    nq_ = sz.nq;
    nv_ = sz.nv;

    // Create vectors
    c_ = Vector::Zero(n);
    J_ = Matrix::Zero(n, sz.nv);
    Jdot_qdot_ = Vector::Zero(n);
}

void Constraint::Update(const ConfigurationVector &q, const TangentVector &v) {
    // Perform callback
    if (callback_ != nullptr) {
        callback_(q, v, c_, J_, Jdot_qdot_);
    } else {
        throw std::runtime_error("Constraint callback is null");
    }
}
