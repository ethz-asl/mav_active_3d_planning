#include "active_3d_planning_core/module/trajectory_evaluator/next_selector/random_complete.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace active_3d_planning {
namespace next_selector {

// RandomComplete
ModuleFactoryRegistry::Registration<RandomComplete>
    RandomComplete::registration("RandomComplete");

RandomComplete::RandomComplete(PlannerI& planner) : NextSelector(planner) {}

void RandomComplete::setupFromParamMap(Module::ParamMap* param_map) {}

// This is super hacky because for some reason it returns an int and not a
// trajsegment* (which would allow us to select w/e a lot easier)
int RandomComplete::selectNextBest(TrajectorySegment* traj_in) {
  std::vector<int> candidates;
  std::vector<TrajectorySegment*> bestSegment;
  for (int i = 0; i < traj_in->children.size(); ++i) {
    auto recursiveChilds = getFromSingle(traj_in->children[i].get());
    for (auto&& child : recursiveChilds) {
      candidates.push_back(i);
      bestSegment.push_back(child);
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
  while (tmptraj->parent) {
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

std::vector<TrajectorySegment*> RandomComplete::getFromSingle(
    TrajectorySegment* traj_in) {
  // Recursively find highest value
  std::vector<TrajectorySegment*> res;
  for (int i = 0; i < traj_in->children.size(); ++i) {
    auto tmp = getFromSingle(traj_in->children[i].get());
    res.insert(res.end(), tmp.begin(), tmp.end());
  }
  return res;
}

}  // namespace next_selector
}  // namespace active_3d_planning
