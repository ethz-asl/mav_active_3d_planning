#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/random_next.h"

#include <algorithm>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// RandomNext
ModuleFactoryRegistry::Registration<RandomNext> RandomNext::registration(
    "RandomNext");

RandomNext::RandomNext(PlannerI& planner) : NextSelector(planner) {}

void RandomNext::setupFromParamMap(Module::ParamMap* param_map) {}

int RandomNext::selectNextBest(TrajectorySegment* traj_in) {
  std::vector<int> candidates = {0};
  for (int i = 1; i < traj_in->children.size(); ++i) {
    candidates.push_back(i);
  }
  // randomize
  std::random_shuffle(candidates.begin(), candidates.end());
  return candidates[0];
}

}  // namespace next_selector
}  // namespace active_3d_planning
