#include "active_3d_planning_core/module/trajectory_evaluator/evaluator_updater/reset_tree.h"

namespace active_3d_planning {
namespace evaluator_updater {

// ResetTree
ModuleFactoryRegistry::Registration<ResetTree> ResetTree::registration(
    "ResetTree");

bool ResetTree::updateSegment(TrajectorySegment* segment) { return false; }

ResetTree::ResetTree(PlannerI& planner) : EvaluatorUpdater(planner) {}

void ResetTree::setupFromParamMap(Module::ParamMap* /*param_map*/) {}

}  // namespace evaluator_updater
}  // namespace active_3d_planning
