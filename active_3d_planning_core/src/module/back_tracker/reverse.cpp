#include "active_3d_planning_core/module/back_tracker/reverse.h"

#include <string>

#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace back_tracker {

// Reverse
ModuleFactoryRegistry::Registration<Reverse> Reverse::registration("Reverse");

Reverse::Reverse(PlannerI& planner) : BackTracker(planner) {}

bool Reverse::segmentIsExecuted(const TrajectorySegment& segment) {
  // Don't track backtracking segments
  if (segment.trajectory.back().position_W == last_position_ &&
      segment.trajectory.back().getYaw() == last_yaw_) {
    return true;
  }
  // Register segments
  if (stack_.size() == stack_size_) {
    for (int i = 0; i < stack_size_ - 1; ++i) {
      stack_[i] = stack_[i + 1];
    }
    stack_[stack_size_ - 1] = segment.trajectory;
  } else {
    stack_.push_back(segment.trajectory);
  }
  return true;
}

bool Reverse::trackBack(TrajectorySegment* target) {
  // Maximum rotations in place reached, time to reverse last segment
  if (stack_.empty()) {
    // Nothing to reverse
    return false;
  } else {
    // Reverse last trajectory
    return reverse(target);
  }
}

bool Reverse::reverse(TrajectorySegment* target) {
  // Reverse the last registered segment.
  EigenTrajectoryPointVector to_reverse = stack_.back();
  std::reverse(to_reverse.begin(), to_reverse.end());
  int64_t current_time = 0;
  TrajectorySegment* new_segment = target->spawnChild();
  for (int i = 1; i < to_reverse.size(); ++i) {
    EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = to_reverse[i].position_W;
    trajectory_point.setFromYaw(
        defaults::angleScaled(to_reverse[i].getYaw() + M_PI));
    current_time +=
        to_reverse[i - 1].time_from_start_ns - to_reverse[i].time_from_start_ns;
    trajectory_point.time_from_start_ns = current_time;
    new_segment->trajectory.push_back(trajectory_point);
  }
  last_yaw_ = new_segment->trajectory.back().getYaw();
  last_position_ = new_segment->trajectory.back().position_W;
  stack_.pop_back();
  return true;
}

void Reverse::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<int>(param_map, "stack_size", &stack_size_, 10);
  stack_.reserve(stack_size_);
}

bool Reverse::checkParamsValid(std::string* error_message) {
  return BackTracker::checkParamsValid(error_message);
}

}  // namespace back_tracker
}  // namespace active_3d_planning
