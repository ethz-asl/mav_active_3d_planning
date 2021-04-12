#include "active_3d_planning_core/module/trajectory_generator/segment_selector/greedy_selector.h"

#include <algorithm>
#include <random>
#include <vector>

#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace segment_selector {

// Greedy
ModuleFactoryRegistry::Registration<GreedySelector>
    GreedySelector::registration("GreedySelector");

GreedySelector::GreedySelector(PlannerI& planner) : SegmentSelector(planner) {}

bool GreedySelector::selectSegment(TrajectorySegment** result,
                                   TrajectorySegment* root) {
  std::vector<TrajectorySegment*> candidates;
  if (leaves_only_) {
    root->getLeaves(&candidates);
  } else {
    root->getTree(&candidates);
  }
  *result = *std::max_element(candidates.begin(), candidates.end(),
                              TrajectorySegment::comparePtr);
  return true;
}

void GreedySelector::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "leaves_only", &leaves_only_, false);
}

}  // namespace segment_selector
}  // namespace active_3d_planning
