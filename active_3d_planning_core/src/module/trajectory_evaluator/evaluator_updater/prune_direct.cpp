#include "active_3d_planning_core/module/trajectory_evaluator/evaluator_updater/prune_direct.h"

#include <string>

#include <active_3d_planning_core/module/module_factory.h>
#include <active_3d_planning_core/planner/planner_I.h>

namespace active_3d_planning {
namespace evaluator_updater {

// PruneByValue
ModuleFactoryRegistry::Registration<PruneDirect> PruneDirect::registration(
    "PruneDirect");

PruneDirect::PruneDirect(PlannerI& planner) : EvaluatorUpdater(planner) {}

void PruneDirect::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "minimum_value", &p_minimum_value_, -1e9);
  setParam<double>(param_map, "minimum_gain", &p_minimum_gain_, -1e9);
  setParam<double>(param_map, "maximum_cost", &p_maximum_cost_, 1e9);

  // Create Following updater (default does nothing)
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
}

bool PruneDirect::updateSegment(TrajectorySegment* segment) {
  if (segment->value < p_minimum_value_ || segment->gain < p_minimum_gain_ ||
      segment->cost > p_maximum_cost_) {
    return false;
  }
  return following_updater_->updateSegment(segment);
}

}  // namespace evaluator_updater
}  // namespace active_3d_planning
