#include "active_3d_planning_core/module/trajectory_evaluator/value_computer/trivial_gain.h"

namespace active_3d_planning {
namespace value_computer {

// TrivialGain
ModuleFactoryRegistry::Registration<TrivialGain> TrivialGain::registration(
    "TrivialGain");

TrivialGain::TrivialGain(PlannerI& planner) : ValueComputer(planner) {}

bool TrivialGain::computeValue(TrajectorySegment* traj_in) {
  traj_in->value = traj_in->gain;
  return true;
}

void TrivialGain::setupFromParamMap(Module::ParamMap* param_map) {}

}  // namespace value_computer
}  // namespace active_3d_planning
