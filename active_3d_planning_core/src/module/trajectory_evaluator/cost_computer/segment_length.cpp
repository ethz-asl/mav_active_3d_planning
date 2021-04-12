#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/segment_length.h"

namespace active_3d_planning {
namespace cost_computer {
// SegmentLength
ModuleFactoryRegistry::Registration<SegmentLength> SegmentLength::registration(
    "SegmentLength");

SegmentLength::SegmentLength(PlannerI& planner) : CostComputer(planner) {}

void SegmentLength::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
}

bool SegmentLength::computeCost(TrajectorySegment* traj_in) {
  if (traj_in->trajectory.size() < 2) {
    traj_in->cost = 0.0;
    return false;
  }
  // Linear path interpolation
  double distance = 0.0;
  Eigen::Vector3d last_point = traj_in->trajectory[0].position_W;
  for (int i = 1; i < traj_in->trajectory.size(); ++i) {
    distance += (traj_in->trajectory[i].position_W - last_point).norm();
    last_point = traj_in->trajectory[i].position_W;
  }
  traj_in->cost = distance;
  if (p_accumulate_ && traj_in->parent) {
    traj_in->cost += traj_in->parent->cost;
  }
  return true;
}

}  // namespace cost_computer
}  // namespace active_3d_planning
