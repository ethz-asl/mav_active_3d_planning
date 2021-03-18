#include "active_3d_planning_core/module/trajectory_evaluator/evaluator_updater/update_all.h"

#include <string>

#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace evaluator_updater {

// UpdateAll
ModuleFactoryRegistry::Registration<UpdateAll> UpdateAll::registration(
    "UpdateAll");

UpdateAll::UpdateAll(PlannerI& planner) : EvaluatorUpdater(planner) {}

bool UpdateAll::updateSegment(TrajectorySegment* segment) {
  if (update_gain_) {
    planner_.getTrajectoryEvaluator().computeGain(segment);
  }
  if (update_cost_) {
    planner_.getTrajectoryEvaluator().computeCost(segment);
  }
  if (update_value_) {
    planner_.getTrajectoryEvaluator().computeValue(segment);
  }
  return following_updater_->updateSegment(segment);
}

void UpdateAll::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "update_gain", &update_gain_, true);
  setParam<bool>(param_map, "update_cost", &update_cost_, true);
  setParam<bool>(param_map, "update_value", &update_value_, true);

  // Create Following updater (default does nothing)
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
}

}  // namespace evaluator_updater
}  // namespace active_3d_planning
