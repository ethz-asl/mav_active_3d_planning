#include <active_3d_planning_core/module/back_tracker.h>
#include <active_3d_planning_core/planner/planner_I.h>

namespace active_3d_planning {

BackTracker::BackTracker(PlannerI& planner) : Module(planner) {}

bool BackTracker::segmentIsExecuted(const TrajectorySegment& segment) {
  return false;  // Default does not register movement
}
}  // namespace active_3d_planning
