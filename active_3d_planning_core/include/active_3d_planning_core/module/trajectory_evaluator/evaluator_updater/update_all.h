#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_UPDATE_ALL_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_UPDATE_ALL_H_

#include <memory>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Update gain/cost/value for the complete trajectory tree
class UpdateAll : public EvaluatorUpdater {
 public:
  explicit UpdateAll(PlannerI& planner);  // NOLINT

  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<UpdateAll> registration;

  // params
  bool update_gain_;
  bool update_cost_;
  bool update_value_;
  std::unique_ptr<EvaluatorUpdater> following_updater_;
};
}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_UPDATE_ALL_H_
