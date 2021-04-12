#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace next_selector {

// Select the child node which contains the highest value segment in its subtree
class SubsequentBest : public NextSelector {
 public:
  explicit SubsequentBest(PlannerI& planner);  // NOLINT

  // override virtual functions
  int selectNextBest(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<SubsequentBest> registration;

  // methods
  double evaluateSingle(TrajectorySegment* traj_in);
};

}  // namespace next_selector
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_H_
