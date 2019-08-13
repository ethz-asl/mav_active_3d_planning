#include "active_3d_planning/module/trajectory_evaluator/value_computer/trivial_gain.h"

#include <cmath>

namespace active_3d_planning {
namespace value_computer {

// TrivialGain
ModuleFactoryRegistry::Registration<TrivialGain>
    TrivialGain::registration("TrivialGain");

TrivialGain::TrivialGain(PlannerI &planner) : ValueComputer(planner){};

bool TrivialGain::computeValue(TrajectorySegment *traj_in) {
  traj_in->value = traj_in->gain;
  return true;
}

void TrivialGain::setupFromParamMap(Module::ParamMap *param_map) {
  // Create Following value computer
  std::string args; // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_value_computer_args", &args,
                        param_ns + "/following_value_computer");
}

} // namespace value_computer
} // namespace active_3d_planning
