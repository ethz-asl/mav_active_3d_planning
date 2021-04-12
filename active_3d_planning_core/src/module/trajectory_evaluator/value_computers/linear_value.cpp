#include "active_3d_planning_core/module/trajectory_evaluator/value_computer/linear_value.h"

#include <cmath>

namespace active_3d_planning {
namespace value_computer {

// LinearValue
ModuleFactoryRegistry::Registration<LinearValue> LinearValue::registration(
    "LinearValue");

LinearValue::LinearValue(PlannerI& planner) : ValueComputer(planner) {}

bool LinearValue::computeValue(TrajectorySegment* traj_in) {
  double gain = traj_in->gain;
  double cost = traj_in->cost;
  if (p_accumulate_cost_) {
    // accumulate cost up to the root
    TrajectorySegment* current = traj_in->parent;
    while (current) {
      cost += current->cost;
      current = current->parent;
    }
  }
  if (p_accumulate_gain_) {
    // accumulate gain up to the root
    TrajectorySegment* current = traj_in->parent;
    while (current) {
      gain += current->gain;
      current = current->parent;
    }
  }
  traj_in->value = gain_weight_ * gain - cost_weight_ * cost;
  return true;
}

void LinearValue::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "cost_weight", &cost_weight_, 1.0);
  setParam<double>(param_map, "gain_weight", &gain_weight_, 1.0);
  setParam<bool>(param_map, "accumulate_cost", &p_accumulate_cost_, false);
  setParam<bool>(param_map, "accumulate_gain", &p_accumulate_gain_, false);
}
}  // namespace value_computer
}  // namespace active_3d_planning
