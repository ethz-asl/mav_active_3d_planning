#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_SIMULATED_SENSOR_UPDATER_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_SIMULATED_SENSOR_UPDATER_H_

#include <memory>

#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {
using SimulatedSensorEvaluator =
    active_3d_planning::trajectory_evaluator::SimulatedSensorEvaluator;

// Updater specific for simulated sensor evaluators. Calls the evaluators
// computeGainFromVisibleVoxels function
// -> no need for raycasting again.
class SimulatedSensorUpdater : public EvaluatorUpdater {
 public:
  explicit SimulatedSensorUpdater(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<SimulatedSensorUpdater>
      registration;

  // members
  std::unique_ptr<EvaluatorUpdater> following_updater_;
  SimulatedSensorEvaluator* evaluator_;
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_SIMULATED_SENSOR_UPDATER_H_
