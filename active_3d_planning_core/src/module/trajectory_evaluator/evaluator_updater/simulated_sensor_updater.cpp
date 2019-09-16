#include "active_3d_planning/module/trajectory_evaluator/evaluator_updater/simulated_sensor_updater.h"

#include "active_3d_planning/module/module_factory.h"
#include "active_3d_planning/planner/planner_I.h"

namespace active_3d_planning {
namespace evaluator_updater {

// SimulatedSensorUpdater
ModuleFactoryRegistry::Registration<SimulatedSensorUpdater>
    SimulatedSensorUpdater::registration("SimulatedSensorUpdater");

SimulatedSensorUpdater::SimulatedSensorUpdater(PlannerI &planner)
    : EvaluatorUpdater(planner) {}

bool SimulatedSensorUpdater::updateSegments(TrajectorySegment *root) {
  // recursively update all segments from root to leaves (as in the planner)
  updateSingle(root);
  return following_updater_->updateSegments(root);
}

void SimulatedSensorUpdater::setupFromParamMap(Module::ParamMap *param_map) {
  // Create Following updater (default does nothing)
  std::string args; // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<EvaluatorUpdater>(
      args, planner_, verbose_modules_);
  evaluator_ = static_cast<SimulatedSensorEvaluator *>(
      planner_.getFactory().readLinkableModule("SimulatedSensorEvaluator"));
}

void SimulatedSensorUpdater::updateSingle(TrajectorySegment *segment) {
  // call the parent to reevaluate the voxels
  // somehow should sanity check correct type
  evaluator_->computeGainFromVisibleVoxels(segment);
  for (int i = 0; i < segment->children.size(); ++i) {
    updateSingle(segment->children[i].get());
  }
}

} // namespace evaluator_updater
} // namespace active_3d_planning
