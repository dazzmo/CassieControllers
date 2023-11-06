#ifndef INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP
#define INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP
/**
 * @file joint_limits_task.h
 * @author dazzmo
 * @brief Task created to avoid joint limits, with soft buffers to encourage accelerations which stray joints from the
 * hard limits rather than acting only when the joints are almost at their limits
 * @version 0.1
 * @date 18-10-2023
 *
 */

#include "controllers/osc/tasks/task.h"

namespace controller {
namespace osc {

class JointLimitsTask : public Task {
   public:
    JointLimitsTask(const std::string &name, const DynamicModel::Size &sz);

    void SetUpperPositionLimit(const ConfigurationVector &qmax) { qu_ = qmax; }
    void SetLowerPositionLimit(const ConfigurationVector &qmin) { ql_ = qmin; }

    void UpdateTask(const Vector &q, const Vector &v);

   private:
    double beta_ = 1.0;
    Vector zeta_;
    ConfigurationVector ql_;
    ConfigurationVector qu_;

    double TransitionFunction(double q, double qmin, double qmax);
};

}  // namespace osc
}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_JOINT_LIMITS_TASK_HPP */
