#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/back_tracker/rotate_in_place.h"

#include <cmath>
#include <string>

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace back_tracker {

// RotateInPlace
ModuleFactoryRegistry::Registration<RotateInPlace> RotateInPlace::registration(
    "RotateInPlace");

RotateInPlace::RotateInPlace(PlannerI& planner) : BackTracker(planner) {}

bool RotateInPlace::trackBack(TrajectorySegment* target) {
  // Just rotate the specified amount
  TrajectorySegment* new_segment = target->spawnChild();
  double yaw = target->trajectory.back().getYaw();
  for (int i = 0; i < std::ceil(sampling_rate_ / update_rate_); ++i) {
    EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = target->trajectory.back().position_W;
    yaw += turn_rate_ / sampling_rate_;
    trajectory_point.setFromYaw(defaults::angleScaled(yaw));
    trajectory_point.time_from_start_ns =
        static_cast<int64_t>(static_cast<double>(i) / sampling_rate_ * 1.0e9);
    new_segment->trajectory.push_back(trajectory_point);
  }
  return true;
}

void RotateInPlace::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "turn_rate", &turn_rate_, 0.79);
  setParam<double>(param_map, "sampling_rate", &sampling_rate_, 20.0);
  setParam<double>(param_map, "update_rate", &update_rate_, 2.0);
}

bool RotateInPlace::checkParamsValid(std::string* error_message) {
  if (sampling_rate_ <= 0.0) {
    *error_message = "sampling_rate expected > 0";
    return false;
  } else if (update_rate_ <= 0.0) {
    *error_message = "update_rate expected > 0";
    return false;
  }
  return BackTracker::checkParamsValid(error_message);
}

}  // namespace back_tracker
}  // namespace active_3d_planning
