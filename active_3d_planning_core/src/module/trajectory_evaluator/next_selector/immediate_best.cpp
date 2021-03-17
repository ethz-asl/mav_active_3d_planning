#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/immediate_best.h"

#include <algorithm>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// ImmediateBest
ModuleFactoryRegistry::Registration<ImmediateBest> ImmediateBest::registration(
    "ImmediateBest");

ImmediateBest::ImmediateBest(PlannerI& planner) : NextSelector(planner) {}

void ImmediateBest::setupFromParamMap(Module::ParamMap* param_map) {}

int ImmediateBest::selectNextBest(TrajectorySegment* traj_in) {
  std::vector<int> candidates = {0};
  double current_max = traj_in->children[0]->value;
  for (int i = 1; i < traj_in->children.size(); ++i) {
    if (traj_in->children[i]->value > current_max) {
      current_max = traj_in->children[i]->value;
      candidates.clear();
      candidates.push_back(i);
    } else if (traj_in->children[i]->value == current_max) {
      candidates.push_back(i);
    }
  }
  // randomize if multiple maxima
  std::random_shuffle(candidates.begin(), candidates.end());
  return candidates[0];
}

}  // namespace next_selector
}  // namespace active_3d_planning
