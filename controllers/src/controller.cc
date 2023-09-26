#include "controllers/controller.h"

Controller::Controller(int nq, int nv, int nu) {
    LOG(INFO) << "Controller constructor";
    nq_ = nq;
    nv_ = nv;
    nu_ = nu;

    qpos_ = Eigen::VectorXd::Zero(nq);
    qvel_ = Eigen::VectorXd::Zero(nv);
    qacc_ = Eigen::VectorXd::Zero(nv);

    qpos_bl_ = Eigen::VectorXd::Zero(nq);
    qpos_bu_ = Eigen::VectorXd::Zero(nq);

    qvel_max_ = Eigen::VectorXd::Zero(nv);
    u_max_ = Eigen::VectorXd::Zero(nu);
}