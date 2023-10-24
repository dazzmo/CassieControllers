#ifndef INCLUDE_CONTROLLERS_CONTROLLER_HPP
#define INCLUDE_CONTROLLERS_CONTROLLER_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <iostream>

enum class TorquePreScale {
    RAMP_UP,
    RAMP_DOWN,
    UNITY
};

enum class ControllerStatus : int {
    SUCCESS,
    FAILURE
};

class Controller {
   public:
    Controller(int nq, int nv, int nu);
    ~Controller() = default;

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

    Eigen::VectorXd& qacc_max() { return qacc_max_; }
    const Eigen::VectorXd& qacc_max() const { return qacc_max_; }

    Eigen::VectorXd& ctrl_max() { return u_max_; }
    const Eigen::VectorXd& ctrl_max() const { return u_max_; }

    Eigen::VectorXd& qpos_0() { return qpos_0_; }
    const Eigen::VectorXd& qpos_0() const { return qpos_0_; }

    void SetCurrentTime(double t) { t_ = t; }
    const double& CurrentTime() const { return t_; }

    void Resize(int nq, int nv, int nu);

    void SetControlFrequency(double freq) { freq_ = freq; }

    void UpdateState(const Eigen::VectorXd& qpos, const Eigen::VectorXd& qvel) {
        qpos_ = qpos;
        qvel_ = qvel;
    }

    ControllerStatus Init();
    void Update(double t);

   protected:
    // Size of configuration vector
    int nq_;
    // Size of tangent vector
    int nv_;
    // Number of control inputs
    int nu_;

    // Time from controller start
    double t_;

    // Control freqency (Hz)
    double freq_;

    // First solve
    bool first_solve_ = true;

    Eigen::VectorXd u_;
    Eigen::VectorXd qpos_;
    Eigen::VectorXd qvel_;
    Eigen::VectorXd qacc_;

    Eigen::VectorXd qpos_0_;

    Eigen::VectorXd qpos_bl_;
    Eigen::VectorXd qpos_bu_;
    Eigen::VectorXd qvel_max_;
    Eigen::VectorXd qacc_max_;
    Eigen::VectorXd u_max_;

    void StartTorqueRampUp(double tau);
    void StartTorqueRampDown(double tau);

    double ApplyTorquePreScale(void);

    virtual void SetupController() {}

    virtual int UpdateControl() {
        throw std::runtime_error("UpdateControl has not been implemented!");
        return 1;
    }

    virtual void UpdateDynamics() {}

   private:
    double ramp_tau_ = 1.0;
    double t_ramp_start_ = 0.0;

    TorquePreScale u_prescale_type_;

    double RampUp(void);
    double RampDown(void);
};

#endif /* INCLUDE_CONTROLLERS_CONTROLLER_HPP */
