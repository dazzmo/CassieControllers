#ifndef INCLUDE_CONTROLLERS_Constraint_HPP
#define INCLUDE_CONTROLLERS_Constraint_HPP

#include <glog/logging.h>

#include <eigen3/Eigen/Core>

#include "controllers/model.h"
#include "controllers/types.h"

namespace controller {

class Constraint {
   public:
    typedef void (*ConstraintCallbackFunction)(const Vector &q, const Vector &v,
                                               Vector &c, Matrix &J, Vector &dJdq);

    Constraint(const std::string &name, Dimension n, const DynamicModel::Size &sz);
    Constraint(const std::string &name, Dimension n, const DynamicModel::Size &sz, ConstraintCallbackFunction callback);
    ~Constraint() = default;

    /**
     * @brief Dimension of the Constraint
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
    const Vector &c() const { return c_; }

    /**
     * @brief Constraint jacobian (ndim x nv)
     *
     */
    const Matrix &J() const { return J_; }

    /**
     * @brief Constraint jacobian time derivative with velocity (ndim x 1)
     *
     */
    const Vector &dJdq() const { return dJdq_; }

    /**
     * @brief Constraint forces
     *
     * @return const Vector&
     */
    const Vector &lambda() const { return lambda_; }

    /**
     * @brief Constraint forces
     *
     * @return Vector&
     */
    Vector &lambda() { return lambda_; }

    void SetStartIndex(Index idx) { start_ = idx; }

    /**
     * @brief Starting index in constraint vector
     *
     * @return const Index
     */
    const Index start() const { return start_; }

    virtual void Update(const ConfigurationVector &q, const TangentVector &v);

   protected:
    Dimension dim_;
    Dimension nq_;
    Dimension nv_;

    // Indexing
    Index start_;

    Vector c_;     // Constraint
    Matrix J_;     // Constraint jacobian
    Vector dJdq_;  // Constraint jacobian time derivative and velocity product

    Vector lambda_;  // Constraint force

   private:
    std::string name_;

    ConstraintCallbackFunction callback_;

    void Resize(Dimension n, const DynamicModel::Size &sz);
};

}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_Constraint_HPP */
