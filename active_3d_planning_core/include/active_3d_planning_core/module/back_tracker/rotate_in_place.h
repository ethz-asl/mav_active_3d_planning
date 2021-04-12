#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_ROTATE_IN_PLACE_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_ROTATE_IN_PLACE_H_

#include <string>

#include "active_3d_planning_core/module/back_tracker.h"

namespace active_3d_planning {
namespace back_tracker {

// Try rotating in place until feasible trajectories were found
class RotateInPlace : public BackTracker {
 public:
  explicit RotateInPlace(PlannerI& planner);  // NOLINT

  // implement virtual functions
  bool trackBack(TrajectorySegment* target) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<RotateInPlace> registration;

  // params
  double turn_rate_;    // rad/s
  double update_rate_;  // Hz, determines length of rotation arc (after which we
  // check for new trajectories)
  double sampling_rate_;  // Hz, trajectory is sampled with sample_rate
};

}  // namespace back_tracker
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_ROTATE_IN_PLACE_H_
