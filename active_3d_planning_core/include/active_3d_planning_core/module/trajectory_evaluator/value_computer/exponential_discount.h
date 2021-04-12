#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_EXPONENTIAL_DISCOUNT_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_EXPONENTIAL_DISCOUNT_H_

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace value_computer {

// Discount the gain with an exponential term of the cost
class ExponentialDiscount : public ValueComputer {
 public:
  explicit ExponentialDiscount(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool computeValue(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<ExponentialDiscount> registration;

  // params
  double p_cost_scale_;
  bool p_accumulate_cost_;  // If true first accumulate all cost, then discount
  bool p_accumulate_gain_;  // If true first accumulate all gain, then discount
};

}  // namespace value_computer
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_EXPONENTIAL_DISCOUNT_H_
