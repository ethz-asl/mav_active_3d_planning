#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_RESET_TREE_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_RESET_TREE_H_

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace evaluator_updater {

// Discard all segments and start from scratch
class ResetTree : public EvaluatorUpdater {
 public:
  explicit ResetTree(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<ResetTree> registration;
};

}  // namespace evaluator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_EVALUATOR_UPDATER_RESET_TREE_H_
