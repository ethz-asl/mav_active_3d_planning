#include "active_3d_planning_core/module/trajectory_evaluator/evaluator_updater/simulated_sensor_updater.h"

#include <string>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace evaluator_updater {

// SimulatedSensorUpdater
ModuleFactoryRegistry::Registration<SimulatedSensorUpdater>
    SimulatedSensorUpdater::registration("SimulatedSensorUpdater");

SimulatedSensorUpdater::SimulatedSensorUpdater(PlannerI& planner)
    : EvaluatorUpdater(planner) {}

bool SimulatedSensorUpdater::updateSegment(TrajectorySegment* segment) {
  // call evaluator to recompute gain without recasting rays
  evaluator_->computeGainFromVisibleVoxels(segment);
  return following_updater_->updateSegment(segment);
}

void SimulatedSensorUpdater::setupFromParamMap(Module::ParamMap* param_map) {
  // Create Following updater (default does nothing)
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
  evaluator_ = static_cast<SimulatedSensorEvaluator*>(
      planner_.getFactory().readLinkableModule("SimulatedSensorEvaluator"));
}

}  // namespace evaluator_updater
}  // namespace active_3d_planning
