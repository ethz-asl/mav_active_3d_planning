#include "active_3d_planning_core/module/trajectory_generator/segment_selector/random_weighted.h"

#include <algorithm>
#include <random>
#include <string>
#include <vector>

namespace active_3d_planning {
namespace segment_selector {

// RandomWeighted
ModuleFactoryRegistry::Registration<RandomWeighted>
    RandomWeighted::registration("RandomWeighted");

RandomWeighted::RandomWeighted(PlannerI& planner) : SegmentSelector(planner) {}

bool RandomWeighted::selectSegment(TrajectorySegment** result,
                                   TrajectorySegment* root) {
  // Get all candidates
  std::vector<TrajectorySegment*> candidates;
  if (static_cast<double>(rand()) / RAND_MAX <= leaf_probability_) {
    root->getLeaves(&candidates);
  } else {
    root->getTree(&candidates);
  }
  if (!revisit_) {
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                                    [](const TrajectorySegment* segment) {
                                      return segment->tg_visited;
                                    }),
                     candidates.end());
  }
  if (candidates.empty()) {
    // exception catching
    *result = root;
    return false;
  }
  if (factor_ > 0.0 &&
      static_cast<double>(rand()) / RAND_MAX > uniform_probability_) {
    // Weighted selection, cdf of every element's value ^ factor
    std::vector<double> values(candidates.size());
    double value_sum = 0.0;
    // shift to 0 to compensate for negative values
    double min_value = (*std::min_element(candidates.begin(), candidates.end(),
                                          TrajectorySegment::comparePtr))
                           ->value;

    // cumulated probability density function
    for (int i = 0; i < candidates.size(); ++i) {
      value_sum += pow((candidates[i]->value - min_value), factor_);
      values[i] = value_sum;
    }
    double realization = static_cast<double>(rand()) / RAND_MAX * value_sum;
    for (int i = 0; i < candidates.size(); ++i) {
      if (values[i] >= realization) {
        *result = candidates[i];
        return true;
      }
    }
  }
  // uniform selection
  *result = candidates[rand() % candidates.size()];
  return true;
}

void RandomWeighted::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "factor", &factor_, 1.0);
  setParam<double>(param_map, "uniform_probability", &uniform_probability_,
                   0.0);
  setParam<double>(param_map, "leaf_probability", &leaf_probability_, 0.0);
  setParam<bool>(param_map, "revisit", &revisit_, false);
}

bool RandomWeighted::checkParamsValid(std::string* error_message) {
  if (uniform_probability_ > 1.0 || uniform_probability_ < 0.0) {
    *error_message = "uniform_probability expected in [0.0, 1.0]";
    return false;
  } else if (leaf_probability_ > 1.0 || leaf_probability_ < 0.0) {
    *error_message = "leaf_probability expected in [0.0, 1.0]";
    return false;
  }
  return true;
}

}  // namespace segment_selector
}  // namespace active_3d_planning
