#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_UPDATE_NOTHING_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_UPDATE_NOTHING_H_

#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace generator_updater {

// Don't perform specific update operations
class UpdateNothing : public GeneratorUpdater {
 public:
  explicit UpdateNothing(PlannerI& planner);  // NOLINT

  bool updateSegment(TrajectorySegment* segment) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<UpdateNothing> registration;
};

}  // namespace generator_updater
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_GENERATOR_UPDATER_UPDATE_NOTHING_H_
