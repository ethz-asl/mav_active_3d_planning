#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/segment_time.h"

namespace active_3d_planning {
namespace cost_computer {

// SegmentTime
ModuleFactoryRegistry::Registration<SegmentTime> SegmentTime::registration(
    "SegmentTime");

SegmentTime::SegmentTime(PlannerI& planner) : CostComputer(planner) {}

void SegmentTime::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
}

bool SegmentTime::computeCost(TrajectorySegment* traj_in) {
  if (traj_in->trajectory.size() < 2) {
    traj_in->cost = 0.0;
    return false;
  }
  traj_in->cost =
      static_cast<double>(traj_in->trajectory.back().time_from_start_ns -
                          traj_in->trajectory.front().time_from_start_ns) *
      1.0e-9;
  if (p_accumulate_ && traj_in->parent) {
    traj_in->cost += traj_in->parent->cost;
  }
  return true;
}

}  // namespace cost_computer
}  // namespace active_3d_planning
