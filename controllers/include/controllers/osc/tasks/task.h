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
                                         Vector &x, Matrix &J, Vector &dJdt_v);

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
    const Vector &dJdt_v() const { return dJdt_v_; }

    /**
     * @brief Proportional gains for error output
     *
     */
    const DiagonalMatrix &Kp() const { return Kp_; }

    /**
     * @brief Derivative gains for error output
     *
     */
    const DiagonalMatrix &Kd() const { return Kd_; }

    /**
     * @brief Set the diagonal of the Kp matrix for error output calculation
     * 
     * @param Kp_vec 
     */
    void SetKpGains(const Vector &Kp_vec) { Kp_.diagonal() = Kp_vec; }

    /**
     * @brief Set the diagonal of the Kd matrix for error output calculation
     * 
     * @param Kd_vec 
     */
    void SetKdGains(const Vector &Kd_vec) { Kd_.diagonal() = Kd_vec; }

    /**
     * @brief Diagonal matrix for task weight matrix
     *
     * @return const DiagonalMatrix&
     */
    const DiagonalMatrix &TaskWeightMatrix() const { return W_; }

    /**
     * @brief Sets the diagonal of the weighting matrix W
     * 
     * @param W 
     */
    void SetTaskWeightMatrix(const Vector &W_vec) { W_.diagonal() = W_vec; }

    void SetReference(const Vector &r);
    void SetReference(const Vector &r, const Vector &dr);
    void SetReference(const Vector &r, const Vector &dr, const Vector &ddr);

    /**
     * @brief Returns the PD error e = Kp (x - r) + Kd (dx - dr)
     *
     * @return const Vector&
     */
    const Vector &ErrorOutputPD() const { return pd_out_; }

    /**
     * @brief Sets the starting index of this task within the task vector
     *
     * @param idx
     */
    void SetStartIndex(Index idx) { start_ = idx; }

    /**
     * @brief Starting index in task vector
     *
     * @return const Index
     */
    const Index start() const { return start_; }

    virtual void Update(const ConfigurationVector &q, const TangentVector &v);

   protected:
    Dimension dim_;

    Vector x_;    // Task
    Vector dx_;   // Task velocity
    Vector ddx_;  // Task acceleration

    Vector r_;    // Reference task
    Vector dr_;   // Reference task rate
    Vector ddr_;  // Reference task acceleration

    Vector e_;    // Error
    Vector de_;   // Error rate
    Vector dde_;  // Error acceleration

    Vector pd_out_;  // PD error output

    Matrix J_;       // Task jacobian
    Vector dJdt_v_;  // Task jacobian time derivative and velocity product

   private:
    std::string name_;

    // Starting index in task vector
    Index start_;

    DiagonalMatrix W_;  // Task weighting diagonal matrix

    DiagonalMatrix Kp_;  // Proportional gains for task-error computation
    DiagonalMatrix Kd_;  // Derivative gains for task-error computation

    TaskCallbackFunction callback_;

    void Resize(Dimension n, const DynamicModel::Size &sz);
};

}  // namespace osc
}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_TASK_HPP */
