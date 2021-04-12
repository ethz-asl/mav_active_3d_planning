#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_NO_COST_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_NO_COST_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace cost_computer {

class NoCost : public CostComputer {
 public:
  explicit NoCost(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool computeCost(TrajectorySegment* traj_in) override;

 protected:
  void setupFromParamMap(Module::ParamMap* param_map) override;

  static ModuleFactoryRegistry::Registration<NoCost> registration;
};

}  // namespace cost_computer
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_NO_COST_H_
