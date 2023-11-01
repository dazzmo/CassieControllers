#ifndef INCLUDE_CONTROLLERS_TASK_HPP
#define INCLUDE_CONTROLLERS_TASK_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>

class Task {
   public:
    Task(int dim, int nv, const std::string &name);
    Task(int dim, int nv, const std::string &name, int (*callback)(const double**, double**));
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
    const Eigen::VectorXd &dJdq() const { return dJdq_; }

    /**
     * @brief Task weighting
     *
     */
    double weight() const { return w_; }

    void SetTaskWeighting(double w) { w_ = w; }

    void SetReference(const Eigen::VectorXd &r);
    void SetReference(const Eigen::VectorXd &r, const Eigen::VectorXd &dr);
    void SetReference(const Eigen::VectorXd &r, const Eigen::VectorXd &dr, const Eigen::VectorXd &ddr);

    virtual void SetProportionalErrorGain(const Eigen::VectorXd &Kp) { Kp_ = Kp; }
    virtual void SetDerivativeErrorGain(const Eigen::VectorXd &Kd) { Kd_ = Kd; }

    /**
     * @brief Returns the error in the task (i.e. x - r)
     *
     * @return const Eigen::VectorXd&
     */
    const Eigen::VectorXd TaskError() { return x_ - r_; }

    /**
     * @brief Returns the derivative error in the task (i.e. dx - dr)
     *
     * @return const Eigen::VectorXd&
     */
    const Eigen::VectorXd TaskErrorDerivative() { return dx_ - dr_; }

    /**
     * @brief Returns the PD error of the task, using the gains provided (i.e. Kp (x - r) + Kd (dx - dr))
     *
     * @return const Eigen::VectorXd&
     */
    const Eigen::VectorXd TaskErrorPD() { return (Kp_.asDiagonal() * TaskError() + Kd_.asDiagonal() * TaskErrorDerivative()); };

    void PrintTaskData();

    virtual int UpdateTask(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true);

   protected:
    int dim_;
    int nq_;
    int nv_;
    std::string name_;

    double w_;  // Task weighting

    Eigen::VectorXd x_;    // Task
    Eigen::VectorXd dx_;   // Task rate
    Eigen::VectorXd ddx_;  // Task acceleration

    Eigen::VectorXd r_;    // Reference task
    Eigen::VectorXd dr_;   // Reference task rate
    Eigen::VectorXd ddr_;  // Reference task acceleration

    Eigen::MatrixXd J_;     // Task jacobian
    Eigen::VectorXd dJdq_;  // Task jacobian time derivative with velocity

    Eigen::VectorXd Kp_;  // Proportional gains for task-error computation
    Eigen::VectorXd Kd_;  // Derivative gains for task-error computation

    // Codegen function pointer to compute task, task jacobian and
    // task jacobian time derivative - velocity product. Input convention is (q, v) and output is (x, J, dJdq)
    int (*callback_)(const double **, double **);

   private:
    int Resize(int ndim, int nv);
};

#endif /* INCLUDE_CONTROLLERS_TASK_HPP */
