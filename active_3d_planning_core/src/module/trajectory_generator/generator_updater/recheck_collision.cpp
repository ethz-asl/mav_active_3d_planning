#include "active_3d_planning_core/module/trajectory_generator/generator_updater/recheck_collision.h"

#include <algorithm>
#include <memory>
#include <vector>

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace generator_updater {

// RecheckCollision
ModuleFactoryRegistry::Registration<RecheckCollision>
    RecheckCollision::registration("RecheckCollision");

RecheckCollision::RecheckCollision(PlannerI& planner)
    : GeneratorUpdater(planner) {}

void RecheckCollision::setupFromParamMap(Module::ParamMap* param_map) {}

bool RecheckCollision::updateSegment(TrajectorySegment* segment) {
  for (const EigenTrajectoryPoint& point : segment->trajectory) {
    if (!(planner_.getTrajectoryGenerator().checkTraversable(
            point.position_W))) {
      return false;
    }
  }
  return true;
}

}  // namespace generator_updater
}  // namespace active_3d_planning
