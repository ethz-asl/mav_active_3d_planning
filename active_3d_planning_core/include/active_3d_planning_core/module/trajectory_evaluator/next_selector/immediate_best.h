#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_IMMEDIATE_BEST_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_IMMEDIATE_BEST_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace next_selector {

// Select the child node which has the highest value
class ImmediateBest : public NextSelector {
 public:
  explicit ImmediateBest(PlannerI& planner);  // NOLINT

  // override virtual functions
  int selectNextBest(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<ImmediateBest> registration;
};

}  // namespace next_selector
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_IMMEDIATE_BEST_H_
