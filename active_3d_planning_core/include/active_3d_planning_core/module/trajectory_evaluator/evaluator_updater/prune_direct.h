#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_PRUNE_DIRECT_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_PRUNE_DIRECT_H_

#include <memory>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Remove all segments that dont have a minimum value, can then call another
// updater
class PruneDirect : public EvaluatorUpdater {
 public:
  explicit PruneDirect(PlannerI& planner);  // NOLINT

  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<PruneDirect> registration;

  // params
  double p_minimum_value_;  // value below which segments are cropped
  double p_minimum_gain_;   // gain below which segments are cropped
  double p_maximum_cost_;   // gain below which segments are cropped

  std::unique_ptr<EvaluatorUpdater> following_updater_;
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_PRUNE_DIRECT_H_
