#include "active_3d_planning_mav/module/back_tracker/feasible_rotate_reverse.h"

#include "active_3d_planning_core/data/system_constraints.h"
#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace back_tracker {

ModuleFactoryRegistry::Registration<FeasibleRotateReverse>
    FeasibleRotateReverse::registration("FeasibleRotateReverse");

FeasibleRotateReverse::FeasibleRotateReverse(PlannerI& planner)
    : RotateReverse(planner) {}

void FeasibleRotateReverse::setupFromParamMap(Module::ParamMap* param_map) {
  // Setup parent and segment creator
  RotateReverse::setupFromParamMap(param_map);
  segment_generator_.setConstraints(
      1.0, 1.0, planner_.getSystemConstraints().yaw_rate_max,
      planner_.getSystemConstraints().yaw_accel_max,
      sampling_rate_);  // velocities don't matter
}

bool FeasibleRotateReverse::rotate(TrajectorySegment* target) {
  EigenTrajectoryPoint goal;
  goal.position_W = target->trajectory.back().position_W;
  goal.setFromYaw(defaults::angleScaled(target->trajectory.back().getYaw() +
                                        turn_rate_ / update_rate_));
  EigenTrajectoryPointVector trajectory;
  if (segment_generator_.createTrajectory(target->trajectory.back(), goal,
                                          &trajectory)) {
    TrajectorySegment* new_segment = target->spawnChild();
    new_segment->trajectory = trajectory;
    current_rotation_ += turn_rate_ / update_rate_;
    last_yaw_ = new_segment->trajectory.back().getYaw();
    last_position_ = new_segment->trajectory.back().position_W;
    return true;
  } else {
    // if mav_trajectory_generation failed for some reason use the old
    // implementation to not die
    return RotateReverse::rotate(target);
  }
}

}  // namespace back_tracker
}  // namespace active_3d_planning
