#include "active_3d_planning_core/module/trajectory_evaluator/value_computer/global_normalized_gain.h"

#include <algorithm>

namespace active_3d_planning {
namespace value_computer {

// GlobalNormalizedGain
ModuleFactoryRegistry::Registration<GlobalNormalizedGain>
    GlobalNormalizedGain::registration("GlobalNormalizedGain");

GlobalNormalizedGain::GlobalNormalizedGain(PlannerI& planner)
    : ValueComputer(planner) {}

void GlobalNormalizedGain::setupFromParamMap(Module::ParamMap* param_map) {}

bool GlobalNormalizedGain::computeValue(TrajectorySegment* traj_in) {
  double gain = 0.0;
  double cost = 0.0;
  TrajectorySegment* current = traj_in->parent;
  while (current) {
    // propagate the new value up to the root
    gain += current->gain;
    cost += current->cost;
    current = current->parent;
  }
  traj_in->value = findBest(traj_in, gain, cost);
  return true;
}

double GlobalNormalizedGain::findBest(TrajectorySegment* current, double gain,
                                      double cost) {
  // recursively iterate towards leaf, then iterate backwards and select best
  // value of children
  double value = 0.0;
  gain += current->gain;
  cost += current->cost;
  if (cost > 0) {
    value = gain / cost;
  }
  for (int i = 0; i < current->children.size(); ++i) {
    value = std::max(value, findBest(current->children[i].get(), gain, cost));
  }
  return value;
}

}  // namespace value_computer
}  // namespace active_3d_planning
