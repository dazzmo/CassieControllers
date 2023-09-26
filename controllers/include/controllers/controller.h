#ifndef INCLUDE_CONTROLLERS_CONTROLLER_HPP
#define INCLUDE_CONTROLLERS_CONTROLLER_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <iostream>


class Controller {
   public:
    Controller() = default;
    ~Controller() = default;

    Controller(int nq, int nv, int nu);

    const int& nq() const { return nq_; }
    const int& nv() const { return nv_; }
    const int& nu() const { return nu_; }

    Eigen::VectorXd& qpos() { return qpos_; }
    const Eigen::VectorXd& qpos() const { return qpos_; }

    Eigen::VectorXd& qvel() { return qvel_; }
    const Eigen::VectorXd& qvel() const { return qvel_; }

    Eigen::VectorXd& qacc() { return qacc_; }
    const Eigen::VectorXd& qacc() const { return qacc_; }

    Eigen::VectorXd& ctrl() { return u_; }
    const Eigen::VectorXd& ctrl() const { return u_; }

    Eigen::VectorXd& qpos_bl() { return qpos_bl_; }
    const Eigen::VectorXd& qpos_bl() const { return qpos_bl_; }

    Eigen::VectorXd& qpos_bu() { return qpos_bu_; }
    const Eigen::VectorXd& qpos_bu() const { return qpos_bu_; }

    Eigen::VectorXd& qvel_max() { return qvel_max_; }
    const Eigen::VectorXd& qvel_max() const { return qvel_max_; }

    Eigen::VectorXd& ctrl_max() { return u_max_; }
    const Eigen::VectorXd& ctrl_max() const { return u_max_; }

    void SetControlFrequency(double freq) { freq_ = freq; }

    void UpdateState(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
        qpos_ = qpos;
        qvel_ = qvel;
    }

   protected:
    // Size of configuration vector
    int nq_;
    // Size of tangent vector
    int nv_;
    // Number of control inputs
    int nu_;

    // Control freqency (Hz)
    double freq_;

    Eigen::VectorXd u_;
    Eigen::VectorXd qpos_;
    Eigen::VectorXd qvel_;
    Eigen::VectorXd qacc_;

    Eigen::VectorXd qpos_bl_;
    Eigen::VectorXd qpos_bu_;
    Eigen::VectorXd qvel_max_;
    Eigen::VectorXd u_max_;

   private:
};

#endif /* INCLUDE_CONTROLLERS_CONTROLLER_HPP */
