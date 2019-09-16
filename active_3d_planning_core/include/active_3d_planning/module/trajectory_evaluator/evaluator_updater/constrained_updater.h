#ifndef ACTIVE_3D_PLANNING_EVALUATOR_UPDATER_DEFAULT_EVALUATOR_UPDATER_H
#define ACTIVE_3D_PLANNING_EVALUATOR_UPDATER_DEFAULT_EVALUATOR_UPDATER_H

#include "active_3d_planning/module/module_factory.h"
#include "active_3d_planning/module/trajectory_evaluator.h"

#include <memory>

namespace active_3d_planning {
namespace evaluator_updater {
// Update gain, only if conditions are met (min gain, nearby)
class ConstrainedUpdater : public EvaluatorUpdater {
public:
  ConstrainedUpdater(PlannerI &planner);
  bool updateSegments(TrajectorySegment *root) override;

  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  static ModuleFactoryRegistry::Registration<ConstrainedUpdater> registration;

  // methods
  void updateSingle(TrajectorySegment *segment);

  // params
  double p_minimum_gain_;
  double p_update_range_;
  std::unique_ptr<EvaluatorUpdater> following_updater_;
};

} // namespace evaluator_updater
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_EVALUATOR_UPDATER_DEFAULT_EVALUATOR_UPDATER_H
