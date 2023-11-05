#ifndef INCLUDE_CONTROLLERS_TASK_HPP
#define INCLUDE_CONTROLLERS_TASK_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>

#include "controllers/model.h"
#include "controllers/types.h"

namespace controller {
namespace osc {

class Task {
   public:
    typedef void (*TaskCallbackFunction)(const Vector &q, const Vector &v,
                                         Vector &x, Matrix &J, Vector &dJdq);

    Task(const std::string &name, Dimension n, const DynamicModel::Size &sz);
    Task(const std::string &name, Dimension n, const DynamicModel::Size &sz, TaskCallbackFunction callback);
    ~Task() = default;

    /**
     * @brief Dimension of the task
     *
     */
    const int &dim() const { return dim_; }

    /**
     * @brief Name
     */
    const std::string &name() const { return name_; }

    /**
     * @brief Task
     */
    const Vector &x() const { return x_; }
    /**
     * @brief Task velocity
     *
     */
    const Vector &dx() const { return dx_; }
    /**
     * @brief Task acceleration
     *
     */
    const Vector &ddx() const { return ddx_; }

    /**
     * @brief Task tracking reference
     *
     */
    const Vector &r() const { return r_; }
    /**
     * @brief Task tracking reference velocity
     *
     */
    const Vector &dr() const { return dr_; }
    /**
     * @brief Task tracking reference acceleration
     *
     */
    const Vector &ddr() const { return ddr_; }

    /**
     * @brief Task jacobian (ndim x nv)
     *
     */
    const Matrix &J() const { return J_; }

    /**
     * @brief Task jacobian time derivative with velocity (ndim x 1)
     *
     */
    const Vector &dJdq() const { return dJdq_; }

    /**
     * @brief Proportional gains for error output
     *
     */
    const Vector &Kp() const { return Kp_; }

    /**
     * @brief Derivative gains for error output
     *
     */
    const Vector &Kd() const { return Kd_; }

    /**
     * @brief Task weighting
     *
     */
    double weight() const { return w_; }

    void SetTaskWeighting(double w) { w_ = w; }

    void SetReference(const Vector &r);
    void SetReference(const Vector &r, const Vector &dr);
    void SetReference(const Vector &r, const Vector &dr, const Vector &ddr);

    void SetErrorGains(const Vector &Kp, const Vector &Kd) {
        Kp_ = Kp;
        Kd_ = Kd;
    }

    void SetErrorGains(const Vector &Kp) {
        Kp_ = Kp;
        Kd_.setZero();
    }

    // void PrintTaskData();

    void Update(const ConfigurationVector &q, const TangentVector &v);

   protected:
    Dimension dim_;
    Dimension nq_;
    Dimension nv_;

    Vector x_;    // Task
    Vector dx_;   // Task velocity
    Vector ddx_;  // Task acceleration

    Vector r_;    // Reference task
    Vector dr_;   // Reference task rate
    Vector ddr_;  // Reference task acceleration

    Vector e_;    // Error
    Vector de_;   // Error rate
    Vector dde_;  // Error acceleration

    Vector pd_out_;  // PD error metric

    Matrix J_;     // Task jacobian
    Vector dJdq_;  // Task jacobian time derivative and velocity product

   private:
    std::string name_;

    Scalar w_;  // Task weighting

    Vector Kp_;  // Proportional gains for task-error computation
    Vector Kd_;  // Derivative gains for task-error computation

    TaskCallbackFunction callback_;

    void Resize(Dimension n, const DynamicModel::Size &sz);
};

}  // namespace osc
}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_TASK_HPP */
