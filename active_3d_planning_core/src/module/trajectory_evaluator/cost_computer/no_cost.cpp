#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/no_cost.h"

namespace active_3d_planning {
namespace cost_computer {

// NoCost
ModuleFactoryRegistry::Registration<NoCost> NoCost::registration("NoCost");

NoCost::NoCost(PlannerI& planner) : CostComputer(planner) {}

void NoCost::setupFromParamMap(Module::ParamMap* param_map) {}

bool NoCost::computeCost(TrajectorySegment* traj_in) {
  traj_in->cost = 0.0;
  if (traj_in->trajectory.size() < 2) {
    return false;
  }
  return true;
}

}  // namespace cost_computer
}  // namespace active_3d_planning
