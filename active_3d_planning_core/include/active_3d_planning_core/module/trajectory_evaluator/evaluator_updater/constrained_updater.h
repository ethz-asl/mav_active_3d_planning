#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_CONSTRAINED_UPDATER_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_CONSTRAINED_UPDATER_H_

#include <memory>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Update gain, only if conditions are met (min gain, nearby)
class ConstrainedUpdater : public EvaluatorUpdater {
 public:
  explicit ConstrainedUpdater(PlannerI& planner);  // NOLINT

  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<ConstrainedUpdater> registration;

  // params
  double p_minimum_gain_;
  double p_update_range_;
  std::unique_ptr<EvaluatorUpdater> following_updater_;
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_CONSTRAINED_UPDATER_H_
