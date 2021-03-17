#include "active_3d_planning_core/module/trajectory_generator/generator_updater/update_nothing.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace active_3d_planning {
namespace generator_updater {

ModuleFactoryRegistry::Registration<UpdateNothing> UpdateNothing::registration(
    "UpdateNothingGenerator");

UpdateNothing::UpdateNothing(PlannerI& planner) : GeneratorUpdater(planner) {}

bool UpdateNothing::updateSegment(TrajectorySegment* segment) { return true; }

void UpdateNothing::setupFromParamMap(Module::ParamMap* param_map) {}

}  // namespace generator_updater
}  // namespace active_3d_planning
