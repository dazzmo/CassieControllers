#ifndef INCLUDE_CONTROLLERS_JOINT_TRACK_TASK_HPP
#define INCLUDE_CONTROLLERS_JOINT_TRACK_TASK_HPP

#include "controllers/osc/tasks/task.h"

namespace controller {
namespace osc {

class JointTrackTask : public Task {
   public:
    JointTrackTask(const DynamicModel::Size &sz);
    void Update(const Vector &q, const Vector &v);

   private:
      Dimension nq_;
      Dimension nv_;
};

}  // namespace osc
}  // namespace controller

#endif /* INCLUDE_CONTROLLERS_JOINT_TRACK_TASK_HPP */
