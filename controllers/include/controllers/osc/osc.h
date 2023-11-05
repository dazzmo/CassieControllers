#ifndef OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP
#define OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP

#include <iostream>
#include <map>
#include <memory>
#include <qpOASES.hpp>
#include <utility>

#include "controllers/acc_limits.h"
#include "controllers/controller.h"
#include "controllers/osc/options.h"
#include "controllers/osc/tasks/ee_task.h"
#include "controllers/osc/tasks/joint_limits_task.h"
#include "controllers/osc/tasks/joint_track_task.h"
#include "controllers/osc/tasks/task.h"
#include "controllers/types.h"

namespace controller {
namespace osc {

class OperationalSpaceController : public Controller {
   public:
    OperationalSpaceController();
    ~OperationalSpaceController();

    void AddTask(const std::string& name, Dimension n, Task::TaskCallbackFunction callback);
    void AddEndEffectorTask(const std::string& name, Task::TaskCallbackFunction &callback);

    void SetEndEffectorContact(const std::string& name, double mu, const Eigen::Vector3d& normal);
    void RemoveEndEffectorContact(const std::string& name);

    void AddJointTrackTask(double weight, const Vector& Kp, const Vector& Kd);
    void UpdateJointTrackReference(const ConfigurationVector& q);
    void UpdateJointTrackReference(const ConfigurationVector& q, const TangentVector& v);

    void AddJointLimitsTask(double weight, const Vector& Kp, const Vector& Kd);

    void SetTorqueWeight(double weight) { torque_weight_ = weight; }

    void CreateOSC(const Options& opt = Options());
    const ActuationVector& RunOSC();

    std::shared_ptr<Task> GetTask(const std::string& name) { return tasks_[name]; }
    std::map<std::string, std::shared_ptr<EndEffectorTask>>& GetEndEffectorTaskMap() { return ee_tasks_; }

   protected:
    // Null space projector for holonomic constraints
    Matrix N_;
    // Holonomic constraint jacobian
    Matrix Jh_;
    // Holonomic constraint jacobian derivative velocity product
    Vector dJhdq_;

    // Quadratic programming
    qpOASES::Options qp_opt_;
    std::unique_ptr<qpOASES::SQProblem> qp_;

    struct QPData {
        QPData(int nx, int ng) {
            H = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(nx, nx);
            g = Vector::Zero(nx);
            A = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>::Zero(ng, nx);
            ubA = qpOASES::INFTY * Vector::Ones(ng);
            lbA = -qpOASES::INFTY * Vector::Ones(ng);
            ubx = qpOASES::INFTY * Vector::Ones(nx);
            lbx = -qpOASES::INFTY * Vector::Ones(nx);
            x = Vector::Zero(nx);
        }
        // Solution vector
        Vector x;
        // QP Hessian matrix
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> H;
        // QP gradient vector
        Vector g;
        // QP constraint jacobian
        Eigen::Matrix<double, -1, -1, Eigen::RowMajor> A;
        // QP constraint lower bound
        Vector lbA;
        // QP constraint upper bound
        Vector ubA;
        // QP variables lower bound
        Vector lbx;
        // QP variables upper bound
        Vector ubx;
    };

    QPData* qp_data_;

   private:
    bool osc_setup_ = false;
    bool use_joint_limits_ = false;
    bool use_joint_track_ = false;
    bool hot_start_ = false;

    double torque_weight_ = 1.0;

    struct OptimisationResult {
        OptimisationResult(int nv, int nc, int ng, int nu) {
            qacc = Vector::Zero(nv);
            lambda_c = Vector::Zero(3 * nc);
            lambda_h = Vector::Zero(ng);
            ctrl = Vector::Zero(nu);
        };

        Vector qacc;
        Vector lambda_c;
        Vector lambda_h;
        Vector ctrl;
    };

    OptimisationResult* res_;

    struct StartingIndices {
        StartingIndices() : qacc(0), lambda_c(0), lambda_h(-1), ctrl(0){};

        Index qacc;
        Index lambda_c;
        Index lambda_h;
        Index ctrl;
    };

    StartingIndices start_idx_;

    JointTrackTask* joint_track_task_ = nullptr;
    JointLimitsTask* joint_limits_task_ = nullptr;

    Options* opt_;

    std::map<std::string, std::shared_ptr<Task>> tasks_;
    std::map<std::string, std::shared_ptr<EndEffectorTask>> ee_tasks_;
};

}  // namespace osc
}  // namespace controller

#endif /* OPERATIONAL_SPACE_INCLUDE_OS_CTRL_HPP */
