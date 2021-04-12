#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_COMPLETE_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_COMPLETE_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace next_selector {

struct ValueTrajectoryPair {
  double value;
  TrajectorySegment* traj;

  friend bool operator>(const ValueTrajectoryPair& l,
                        const ValueTrajectoryPair& r) {
    return l.value > r.value;
  }

  friend bool operator<(const ValueTrajectoryPair& l,
                        const ValueTrajectoryPair& r) {
    return l.value < r.value;
  }

  friend bool operator==(const ValueTrajectoryPair& l,
                         const ValueTrajectoryPair& r) {
    return l.value == r.value;
  }
};

// Select the child node which contains the highest value segment in its subtree
class SubsequentBestComplete : public NextSelector {
 public:
  explicit SubsequentBestComplete(PlannerI& planner);  // NOLINT

  // override virtual functions
  int selectNextBest(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<SubsequentBestComplete>
      registration;

  // methods
  ValueTrajectoryPair evaluateSingle(TrajectorySegment* traj_in);
};

}  // namespace next_selector
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_NEXT_SELECTOR_SUBSEQUENT_BEST_COMPLETE_H_
