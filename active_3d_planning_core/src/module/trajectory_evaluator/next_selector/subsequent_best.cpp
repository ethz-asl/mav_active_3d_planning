#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/subsequent_best.h"

#include <algorithm>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// SubsequentBest
ModuleFactoryRegistry::Registration<SubsequentBest>
    SubsequentBest::registration("SubsequentBest");

SubsequentBest::SubsequentBest(PlannerI& planner) : NextSelector(planner) {}

void SubsequentBest::setupFromParamMap(Module::ParamMap* param_map) {}

int SubsequentBest::selectNextBest(TrajectorySegment* traj_in) {
  std::vector<int> candidates = {0};
  double current_max = evaluateSingle(traj_in->children[0].get());
  for (int i = 1; i < traj_in->children.size(); ++i) {
    double current_value = evaluateSingle(traj_in->children[i].get());
    if (current_value > current_max) {
      current_max = current_value;
      candidates.clear();
      candidates.push_back(i);
    } else if (current_value == current_max) {
      candidates.push_back(i);
    }
  }
  // randomize if multiple maxima
  std::random_shuffle(candidates.begin(), candidates.end());
  return candidates[0];
}

double SubsequentBest::evaluateSingle(TrajectorySegment* traj_in) {
  // Recursively find highest value
  if (traj_in->children.empty()) {
    return traj_in->value;
  }
  double highest_value = traj_in->value;
  for (int i = 0; i < traj_in->children.size(); ++i) {
    highest_value =
        std::max(highest_value, evaluateSingle(traj_in->children[i].get()));
  }
  return highest_value;
}

}  // namespace next_selector
}  // namespace active_3d_planning
