#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_ROTATE_REVERSE_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_ROTATE_REVERSE_H_

#include <string>
#include <vector>

#include <active_3d_planning_core/data/trajectory.h>

#include "active_3d_planning_core/module/back_tracker.h"

namespace active_3d_planning {
namespace back_tracker {

// Try rotating in place, if nothing found reverse most recent segments
class RotateReverse : public BackTracker {
 public:
  explicit RotateReverse(PlannerI& planner);  // NOLINT

  // implement virtual functions
  bool segmentIsExecuted(const TrajectorySegment& segment) override;

  bool trackBack(TrajectorySegment* target) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<RotateReverse> registration;

  // params
  double turn_rate_;    // rad/s
  double update_rate_;  // Hz (how often the turn movement is checked for new
  // trajectories)
  double sampling_rate_;  // Hz
  double n_rotations_;
  int stack_size_;

  // variables
  std::vector<EigenTrajectoryPointVector>
      stack_;  // We use a vector for fixed stack size
  Eigen::Vector3d last_position_;
  double last_yaw_;
  double current_rotation_;

  // methods
  virtual bool rotate(TrajectorySegment* target);

  virtual bool reverse(TrajectorySegment* target);
};

}  // namespace back_tracker
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_ROTATE_REVERSE_H_
