#include "active_3d_planning_core/module/trajectory_evaluator/value_computer/accumulate_value.h"

#include <string>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace value_computer {

// AccumulateValue
ModuleFactoryRegistry::Registration<AccumulateValue>
    AccumulateValue::registration("AccumulateValue");

AccumulateValue::AccumulateValue(PlannerI& planner) : ValueComputer(planner) {}

bool AccumulateValue::computeValue(TrajectorySegment* traj_in) {
  following_value_computer_->computeValue(traj_in);
  if (traj_in->parent) {
    traj_in->value += traj_in->parent->value;
  }
  return true;
}

void AccumulateValue::setupFromParamMap(Module::ParamMap* param_map) {
  // Create Following value computer
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_value_computer_args", &args,
                        param_ns + "/following_value_computer");
  following_value_computer_ = planner_.getFactory().createModule<ValueComputer>(
      args, planner_, verbose_modules_);
}

}  // namespace value_computer
}  // namespace active_3d_planning
