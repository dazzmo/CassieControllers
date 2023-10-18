#ifndef INCLUDE_CONTROLLERS_TASK_HPP
#define INCLUDE_CONTROLLERS_TASK_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>

#include "casadi_interface.h"

class Task {
   public:
    Task(int dim, int nv, f_casadi_cg callback);
    ~Task() = default;

    Eigen::VectorXd task_weight;

    /**
     * @brief Dimension of the task
     *
     */
    const int &dim() const { return dim_; }

    /**
     * @brief Task      *
     */
    const Eigen::VectorXd &x() const { return x_; }
    /**
     * @brief Task velocity
     *
     */
    const Eigen::VectorXd &dx() const { return dx_; }
    /**
     * @brief Task acceleration
     *
     */
    const Eigen::VectorXd &ddx() const { return ddx_; }

    /**
     * @brief Task tracking reference
     *
     */
    const Eigen::VectorXd &r() const { return r_; }
    /**
     * @brief Task tracking reference velocity
     *
     */
    const Eigen::VectorXd &dr() const { return dr_; }
    /**
     * @brief Task tracking reference acceleration
     *
     */
    const Eigen::VectorXd &ddr() const { return ddr_; }

    /**
     * @brief Task jacobian (ndim x nv)
     *
     */
    const Eigen::MatrixXd &J() const { return J_; }

    /**
     * @brief Task jacobian time derivative with velocity (ndim x 1)
     *
     */
    const Eigen::VectorXd &dJdv() const { return dJdv_; }

    void SetReference(const Eigen::VectorXd &r);
    void SetReference(const Eigen::VectorXd &r, const Eigen::VectorXd &dr);
    void SetReference(const Eigen::VectorXd &r, const Eigen::VectorXd &dr, const Eigen::VectorXd &ddr);

    virtual int UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true);

   protected:
    int dim_;
    int nv_;

    Eigen::VectorXd x_;    // Task
    Eigen::VectorXd dx_;   // Task rate
    Eigen::VectorXd ddx_;  // Task acceleration

    Eigen::VectorXd r_;    // Reference task
    Eigen::VectorXd dr_;   // Reference task rate
    Eigen::VectorXd ddr_;  // Reference task acceleration

    Eigen::MatrixXd J_;     // Task jacobian
    Eigen::VectorXd dJdv_;  // Task jacobian time derivative with velocity

    // CasADi codegen function pointer to compute task, task jacobian and
    // task jacobian time derivative - velocity product. Input convention is (q, v) and output is (x, J, dJdq)
    f_casadi_cg callback_;

   private:
    int Resize(int ndim, int nv);
};

#endif /* INCLUDE_CONTROLLERS_TASK_HPP */
