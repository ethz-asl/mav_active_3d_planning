#include "active_3d_planning_core/module/trajectory_generator/segment_selector/random_restricted.h"

#include <algorithm>
#include <string>
#include <vector>

namespace active_3d_planning {
namespace segment_selector {

// RandomRestricted
ModuleFactoryRegistry::Registration<RandomRestricted>
    RandomRestricted::registration("RandomRestricted");

RandomRestricted::RandomRestricted(PlannerI& planner)
    : SegmentSelector(planner) {}

bool RandomRestricted::selectSegment(TrajectorySegment** result,
                                     TrajectorySegment* root) {
  // Get all candidates
  std::vector<TrajectorySegment*> candidates;
  root->getTree(&candidates, maxdepth_ - 1);
  if (candidates.empty()) {
    // exception catching
    *result = root;
    return false;
  }
  // uniform selection
  *result = candidates[rand() % candidates.size()];
  return true;
}

void RandomRestricted::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<int>(param_map, "maxdepth", &maxdepth_, 1);
}

bool RandomRestricted::checkParamsValid(std::string* error_message) {
  if (maxdepth_ < 1) {
    *error_message = "maxdepth must be at least 1";
    return false;
  }
  return true;
}

}  // namespace segment_selector
}  // namespace active_3d_planning
