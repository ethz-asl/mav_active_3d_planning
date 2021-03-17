#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/subsequent_best_complete.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// SubsequentBestComplete
ModuleFactoryRegistry::Registration<SubsequentBestComplete>
    SubsequentBestComplete::registration("SubsequentBestComplete");

SubsequentBestComplete::SubsequentBestComplete(PlannerI& planner)
    : NextSelector(planner) {}

void SubsequentBestComplete::setupFromParamMap(Module::ParamMap* param_map) {}

// This is super hacky because for some reason it returns an int and not a
// trajsegment* (which would allow us to select w/e a lot easier)
int SubsequentBestComplete::selectNextBest(TrajectorySegment* traj_in) {
  std::vector<int> candidates = {0};
  std::vector<TrajectorySegment*> bestSegment = {traj_in->children[0].get()};
  double current_max = evaluateSingle(traj_in->children[0].get()).value;
  for (int i = 1; i < traj_in->children.size(); ++i) {
    auto current_value = evaluateSingle(traj_in->children[i].get());
    if (current_value.value > current_max) {
      current_max = current_value.value;
      candidates.clear();
      candidates.push_back(i);
      bestSegment.clear();
      bestSegment.push_back(current_value.traj);
    } else if (current_value.value == current_max) {
      candidates.push_back(i);
      bestSegment.push_back(current_value.traj);
    }
  }
  // randomize if multiple maxima (note this is not perfectly uniform, but shoud
  // do for now)
  int selected = rand() % candidates.size();
  // here we expand the cadidated to include all parts up to the root and update
  // the tree accordingly
  // we do break info here... (TODO require info to overload operator + and fix
  // this)
  std::vector<TrajectorySegment*> allsegs;
  auto tmptraj = bestSegment[selected];
  // parent->parent because we dont want the base segment to be inserted aswell
  // as this get choosen anyways
  while (tmptraj->parent->parent) {
    allsegs.insert(allsegs.begin(), tmptraj);
    tmptraj = tmptraj->parent;
  }

  // add data
  auto nextraj = traj_in->children[candidates[selected]].get();
  for (auto seg : allsegs) {
    auto currtime = nextraj->trajectory.back().time_from_start_ns;
    for (auto&& trajPoint : seg->trajectory) {
      trajPoint.time_from_start_ns += currtime;
      nextraj->trajectory.push_back(trajPoint);
    }
  }
  // adjust children (this will kill alot I think but is fine)
  nextraj->children = std::move(bestSegment[selected]->children);

  return candidates[selected];
}

ValueTrajectoryPair SubsequentBestComplete::evaluateSingle(
    TrajectorySegment* traj_in) {
  // Recursively find highest value
  ValueTrajectoryPair res;
  res.value = traj_in->value;
  res.traj = traj_in;
  for (int i = 0; i < traj_in->children.size(); ++i) {
    res = std::max(res, evaluateSingle(traj_in->children[i].get()));
  }
  return res;
}

}  // namespace next_selector
}  // namespace active_3d_planning
