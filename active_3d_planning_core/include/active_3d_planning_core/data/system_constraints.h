#ifndef ACTIVE_3D_PLANNING_CORE_DATA_SYSTEM_CONSTRAINTS_H_
#define ACTIVE_3D_PLANNING_CORE_DATA_SYSTEM_CONSTRAINTS_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <active_3d_planning_core/module/module_factory_registry.h>
#include <active_3d_planning_core/planner/planner_I.h>

namespace active_3d_planning {

// struct for high level system constraints (might add methods for generating
// these from max thrusts or so)
struct SystemConstraints : public Module {
  explicit SystemConstraints(PlannerI& planner);  // NOLINT

  virtual ~SystemConstraints() = default;

  // populate the system constraints from factory
  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

  // variables
  double v_max;             // m/s, maximum absolute velocity
  double a_max;             // m/s2, maximum absolute acceleration
  double yaw_rate_max;      // rad/s, maximum yaw rate
  double yaw_accel_max;     // rad/s2, maximum yaw acceleration
  double collision_radius;  // m

 protected:
  static ModuleFactoryRegistry::Registration<SystemConstraints> registration;
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_DATA_SYSTEM_CONSTRAINTS_H_
