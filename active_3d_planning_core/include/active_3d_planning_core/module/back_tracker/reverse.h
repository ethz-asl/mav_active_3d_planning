#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_REVERSE_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_REVERSE_H_

#include <string>
#include <vector>

#include <active_3d_planning_core/data/trajectory.h>

#include "active_3d_planning_core/module/back_tracker.h"

namespace active_3d_planning {
namespace back_tracker {

// Try rotating in place, if nothing found reverse most recent segments
class Reverse : public BackTracker {
 public:
  explicit Reverse(PlannerI& planner);  // NOLINT

  // implement virtual functions
  bool segmentIsExecuted(const TrajectorySegment& segment) override;

  bool trackBack(TrajectorySegment* target) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<Reverse> registration;

  // params
  int stack_size_;

  // variables
  std::vector<EigenTrajectoryPointVector>
      stack_;  // We use a vector for fixed stack size
  Eigen::Vector3d last_position_;
  double last_yaw_;
  double current_rotation_;

  // methods
  virtual bool reverse(TrajectorySegment* target);
};

}  // namespace back_tracker
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_REVERSE_H_
