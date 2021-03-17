#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_RECHECK_COLLISION_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_RECHECK_COLLISION_H_

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace generator_updater {

// Recursively check wether the trajectories are still collision free
class RecheckCollision : public GeneratorUpdater {
 public:
  explicit RecheckCollision(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<RecheckCollision> registration;
};

}  // namespace generator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_RECHECK_COLLISION_H_
