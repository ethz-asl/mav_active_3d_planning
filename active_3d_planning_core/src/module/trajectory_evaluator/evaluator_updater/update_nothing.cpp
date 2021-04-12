#include "active_3d_planning_core/module/trajectory_evaluator/evaluator_updater/update_nothing.h"

namespace active_3d_planning {
namespace evaluator_updater {

ModuleFactoryRegistry::Registration<UpdateNothing> UpdateNothing::registration(
    "UpdateNothing");

UpdateNothing::UpdateNothing(PlannerI& planner) : EvaluatorUpdater(planner) {}

bool UpdateNothing::updateSegment(TrajectorySegment* segment) { return true; }

void UpdateNothing::setupFromParamMap(Module::ParamMap* /*param_map*/) {}
}  // namespace evaluator_updater
}  // namespace active_3d_planning
