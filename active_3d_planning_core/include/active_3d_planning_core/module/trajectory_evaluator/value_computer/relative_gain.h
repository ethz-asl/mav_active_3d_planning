#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_RELATIVE_GAIN_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_RELATIVE_GAIN_H_

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace value_computer {

// Computes efficiency as gain divided by cost
class RelativeGain : public ValueComputer {
 public:
  explicit RelativeGain(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool computeValue(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<RelativeGain> registration;

  // params
  bool p_accumulate_;  // true: return total gain divided by total cost
};

// Computes efficiency as gain divided by cost, where future gains are
// discounted per segment
class DiscountedRelativeGain : public ValueComputer {
 public:
  explicit DiscountedRelativeGain(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool computeValue(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<DiscountedRelativeGain>
      registration;

  // params
  double p_discount_factor_;

  // recursively compute the discounted gain sum
  void iterate(TrajectorySegment* current, double* factor, double* gain,
               double* cost);
};

}  // namespace value_computer
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_VALUE_COMPUTER_RELATIVE_GAIN_H_
