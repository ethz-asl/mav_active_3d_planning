#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_H_

#include <active_3d_planning_core/data/trajectory_segment.h>
#include <active_3d_planning_core/module/module_factory_registry.h>

namespace active_3d_planning {

class PlannerI;

// Base class for backtrackers to provide uniform interface with other classes
// A backtracker's responsibility is to handle the states where no new
// trajectories were found for execution.
class BackTracker : public Module {
 public:
  explicit BackTracker(PlannerI& planner);  // NOLINT

  // This function lets the backtracker know when a trajectory is executed
  virtual bool segmentIsExecuted(const TrajectorySegment& segment);

  // This function is executed when no new trajectories are found.
  virtual bool trackBack(TrajectorySegment* target) = 0;
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_BACK_TRACKER_H_
