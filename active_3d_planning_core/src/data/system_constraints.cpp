#include "active_3d_planning_core/data/system_constraints.h"

#include <string>

namespace active_3d_planning {

ModuleFactoryRegistry::Registration<SystemConstraints>
    SystemConstraints::registration("SystemConstraints");

SystemConstraints::SystemConstraints(PlannerI& planner) : Module(planner) {}

void SystemConstraints::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "v_max", &v_max, 1.0);
  setParam<double>(param_map, "a_max", &a_max, 1.0);
  setParam<double>(param_map, "yaw_rate_max", &yaw_rate_max, M_PI / 2.0);
  setParam<double>(param_map, "yaw_accel_max", &yaw_accel_max, M_PI / 2.0);
  setParam<double>(param_map, "collision_radius", &collision_radius, 1.0);
}

bool SystemConstraints::checkParamsValid(std::string* error_message) {
  if (v_max <= 0.0) {
    *error_message = "v_max expected > 0.0";
    return false;
  } else if (a_max <= 0.0) {
    *error_message = "a_max expected > 0.0";
    return false;
  } else if (yaw_rate_max <= 0.0) {
    *error_message = "yaw_rate_max expected > 0.0";
    return false;
  } else if (yaw_accel_max <= 0.0) {
    *error_message = "yaw_accel_max expected > 0.0";
    return false;
  }
  return true;
}

}  // namespace active_3d_planning
