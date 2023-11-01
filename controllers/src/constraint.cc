#include "controllers/constraint.h"

Constraint::Constraint(int dim, int nv, const std::string &name) {
    dim_ = dim;
    nv_ = nv;
    name_ = name;
    callback_ = nullptr;

    Resize(dim_, nv_);
}

Constraint::Constraint(int dim, int nv, const std::string &name, int (*callback)(const double **, double **)) {
    dim_ = dim;
    nv_ = nv;
    name_ = name;
    callback_ = callback;

    Resize(dim_, nv_);
}

int Constraint::Resize(int dim, int nv) {
    c_ = Eigen::VectorXd::Zero(dim);
    J_ = Eigen::MatrixXd::Zero(dim, nv);
    dJdq_ = Eigen::VectorXd::Zero(dim);

    return 0;
}

int Constraint::UpdateConstraint(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians) {
    const double *in[] = {qpos.data(), qvel.data()};
    double *out[3];
    out[0] = c_.data();
    if (update_jacobians) {
        out[1] = J_.data();
        out[2] = dJdq_.data();
        // Use callback
        callback_(in, out);
        // Compute Constraint velocity
        dx_ = J_ * qvel;

    } else {
        out[1] = nullptr;
        out[2] = nullptr;
        callback_(in, out);
    }

    return 0;
}

void Constraint::PrintConstraintData() {
    LOG(INFO) << "Constraint: " << name_ << '\n'
              << "c: " << x_.transpose() << '\n'
              << "J: " << J_ << '\n'
              << "dJdq: " << dJdq_ << '\n';
}
