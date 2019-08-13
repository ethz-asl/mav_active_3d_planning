#include "active_3d_planning/module/trajectory_evaluator/evaluator_updater/prune_by_value.h"

#include <active_3d_planning/planner/planner_I.h>
#include <active_3d_planning/module/module_factory.h>

namespace active_3d_planning {
namespace evaluator_updater {

// PruneByValue
ModuleFactoryRegistry::Registration<PruneByValue>
    PruneByValue::registration("PruneByValue");

PruneByValue::PruneByValue(PlannerI &planner) : EvaluatorUpdater(planner){};

void PruneByValue::setupFromParamMap(Module::ParamMap *param_map) {
  setParam<double>(param_map, "minimum_value", &minimum_value_, 0.0);
  setParam<bool>(param_map, "use_relative_values", &use_relative_values_,
                 false);
  setParam<bool>(param_map, "include_subsequent", &include_subsequent_, true);

  // Create Following updater (default does nothing)
  std::string args; // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
}

bool PruneByValue::updateSegments(TrajectorySegment *root) {
  // Remove all segments, whose value is below the minimum value
  double absolute_minimum = getMinimumValue(root);
  if (include_subsequent_) {
    // remove recursively, all segments where no child is above the minimum
    removeSingle(root, absolute_minimum);
  } else {
    // Remove the immediate children that are below the minimum
    int j = 0;
    for (int i = 0; i < root->children.size(); ++i) {
      if (root->children[j]->value < absolute_minimum) {
        root->children.erase(root->children.begin() + j);
      } else {
        j++;
      }
    }
  }
  return following_updater_->updateSegments(root);
}

double PruneByValue::getMinimumValue(TrajectorySegment *segment) {
  if (use_relative_values_) {
    // Scale from 0 at minimum, 1 at maximum to absolute minimum
    std::vector<TrajectorySegment *> segments;
    if (include_subsequent_) {
      // Get the full tree without the root (which has always 0 value)
      segment->getTree(&segments);
      segments.erase(segments.begin());
    } else {
      segment->getChildren(&segments);
    }
    double min_value = (*std::min_element(segments.begin(), segments.end(),
                                          TrajectorySegment::comparePtr))
                           ->value;
    double max_value = (*std::max_element(segments.begin(), segments.end(),
                                          TrajectorySegment::comparePtr))
                           ->value;
    return minimum_value_ * (max_value - min_value) + min_value;
  } else {
    // The absolute minimum is explicitely set
    return minimum_value_;
  }
}

bool PruneByValue::removeSingle(TrajectorySegment *segment, double min_value) {
  if (segment->children.empty()) {
    return (segment->value < min_value);
  }
  int j = 0;
  for (int i = 0; i < segment->children.size(); ++i) {
    if (removeSingle(segment->children[j].get(), min_value)) {
      segment->children.erase(segment->children.begin() + j);
    } else {
      j++;
    }
  }
  return (segment->children.empty() & segment->value < min_value);
}

} // namespace evaluator_updater
} // namespace active_3d_planning
