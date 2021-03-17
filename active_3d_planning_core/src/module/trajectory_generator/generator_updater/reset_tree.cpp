#include "active_3d_planning_core/module/trajectory_generator/generator_updater/reset_tree.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace active_3d_planning {
namespace generator_updater {

// ResetTree
ModuleFactoryRegistry::Registration<ResetTree> ResetTree::registration(
    "ResetTreeGenerator");

ResetTree::ResetTree(PlannerI& planner) : GeneratorUpdater(planner) {}

void ResetTree::setupFromParamMap(Module::ParamMap* param_map) {}

bool ResetTree::updateSegment(TrajectorySegment* segment) { return false; }

}  // namespace generator_updater
}  // namespace active_3d_planning
