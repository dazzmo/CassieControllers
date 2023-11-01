#ifndef INCLUDE_CONTROLLERS_CONSTRAINT_HPP
#define INCLUDE_CONTROLLERS_CONSTRAINT_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>

class Constraint {
   public:
    Constraint(int dim, int nv, const std::string &name);
    Constraint(int dim, int nv, const std::string &name, int (*callback)(const double **, double **));
    ~Constraint() = default;

    /**
     * @brief Dimension of the constraint
     *
     */
    const int &dim() const { return dim_; }

    /**
     * @brief Name
     */
    const std::string &name() const { return name_; }

    /**
     * @brief Constraint
     */
    const Eigen::VectorXd &c() const { return c_; }

    /**
     * @brief Constraint jacobian (ndim x nv)
     *
     */
    const Eigen::MatrixXd &J() const { return J_; }

    /**
     * @brief Constraint jacobian time derivative with velocity (ndim x 1)
     *
     */
    const Eigen::VectorXd &dJdq() const { return dJdq_; }

    void PrintConstraintData();

    virtual int UpdateConstraint(const Eigen::VectorXd &qpos, const Eigen::VectorXd &qvel, bool update_jacobians = true);

   protected:
    int dim_;
    int nq_;
    int nv_;
    std::string name_;

    Eigen::VectorXd c_;     // Constraint
    Eigen::MatrixXd J_;     // Constraint jacobian
    Eigen::VectorXd dJdq_;  // Constraint jacobian time derivative with velocity

    int (*callback_)(const double **, double **);

   private:
    int Resize(int ndim, int nv);
};

#endif /* INCLUDE_CONTROLLERS_CONSTRAINT_HPP */