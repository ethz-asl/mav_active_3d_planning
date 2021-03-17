#include "active_3d_planning_core/module/trajectory_evaluator/value_computer/relative_gain.h"

#include <cmath>

namespace active_3d_planning {
namespace value_computer {

// RelativeGain
ModuleFactoryRegistry::Registration<RelativeGain> RelativeGain::registration(
    "RelativeGain");

RelativeGain::RelativeGain(PlannerI& planner) : ValueComputer(planner) {}

bool RelativeGain::computeValue(TrajectorySegment* traj_in) {
  double gain = traj_in->gain;
  double cost = traj_in->cost;
  if (p_accumulate_) {
    TrajectorySegment* current = traj_in->parent;
    while (current) {
      gain += current->gain;
      cost += current->cost;
      current = current->parent;
    }
  }
  if (cost == 0.0) {
    traj_in->value = 0.0;
  } else {
    traj_in->value = gain / cost;
  }
  return true;
}

void RelativeGain::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<bool>(param_map, "accumulate", &p_accumulate_, true);
}

// DiscountedRelativeGain
ModuleFactoryRegistry::Registration<DiscountedRelativeGain>
    DiscountedRelativeGain::registration("DiscountedRelativeGain");

DiscountedRelativeGain::DiscountedRelativeGain(PlannerI& planner)
    : ValueComputer(planner) {}

bool DiscountedRelativeGain::computeValue(TrajectorySegment* traj_in) {
  double gain = 0.0;
  double cost = 0.0;
  double factor = 1.0;
  iterate(traj_in, &factor, &gain, &cost);
  if (cost == 0.0) {
    traj_in->value = 0.0;
  } else {
    traj_in->value = gain / cost;
  }
  return true;
}

void DiscountedRelativeGain::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "discount_factor", &p_discount_factor_, 0.9);
}

void DiscountedRelativeGain::iterate(TrajectorySegment* current, double* factor,
                                     double* gain, double* cost) {
  // iterate up to root
  if (current->parent) {
    iterate(current->parent, factor, gain, cost);
  }
  *gain += *factor * current->gain;
  *cost += current->cost;
  *factor *= p_discount_factor_;
}

}  // namespace value_computer
}  // namespace active_3d_planning
