#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_GLOBAL_NORMALIZED_GAIN_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_GLOBAL_NORMALIZED_GAIN_H_

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace value_computer {

// Use the best subsequent value for each segment in the subtree
class GlobalNormalizedGain : public ValueComputer {
 public:
  explicit GlobalNormalizedGain(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool computeValue(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<GlobalNormalizedGain> registration;

  // methods
  double findBest(TrajectorySegment* current, double gain, double cost);
};

}  // namespace value_computer
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_GLOBAL_NORMALIZED_GAIN_H_
