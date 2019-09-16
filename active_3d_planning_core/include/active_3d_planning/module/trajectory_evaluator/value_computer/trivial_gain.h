#ifndef ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_TRIVIAL_GAIN_H
#define ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_TRIVIAL_GAIN_H

#include "active_3d_planning/module/module_factory_registry.h"
#include "active_3d_planning/module/trajectory_evaluator.h"

namespace active_3d_planning {
namespace value_computer {

// Computes the value as the aberag of gains
class TrivialGain : public ValueComputer {
public:
  TrivialGain(PlannerI &planner);
  // override virtual functions
  bool computeValue(TrajectorySegment *traj_in) override;

  void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  static ModuleFactoryRegistry::Registration<TrivialGain> registration;

  // params
  double p_discount_factor_;

  // recursively compute the discounted gain sum
  void iterate(TrajectorySegment *current, double *factor, double *gain,
               double *cost);
};

} // namespace value_computer
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_VALUE_COMPUTER_TRIVIAL_GAIN_H
