#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_TRIVIAL_GAIN_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_TRIVIAL_GAIN_H_

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace value_computer {

// Computes the value as the gain
class TrivialGain : public ValueComputer {
 public:
  explicit TrivialGain(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool computeValue(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<TrivialGain> registration;
};

}  // namespace value_computer
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_TRIVIAL_GAIN_H_
