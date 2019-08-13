#include "active_3d_planning/module/trajectory_generator/generator_updater/recheck_collision.h"

#include "active_3d_planning/data/trajectory.h"
#include "active_3d_planning/planner/planner_I.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace active_3d_planning {
namespace generator_updater {

// RecheckCollision
ModuleFactoryRegistry::Registration<RecheckCollision>
    RecheckCollision::registration("RecheckCollision");

RecheckCollision::RecheckCollision(PlannerI &planner)
    : GeneratorUpdater(planner) {}

void RecheckCollision::setupFromParamMap(Module::ParamMap *param_map) {}

bool RecheckCollision::updateSegments(TrajectorySegment *root) {
  checkSingle(root);
  return true;
}

bool RecheckCollision::isCollided(
    const EigenTrajectoryPointVector &trajectory) {
  for (int i = 0; i < trajectory.size(); ++i) {
    if (!(planner_.getTrajectoryGenerator().checkTraversable(
            trajectory[i].position_W))) {
      return true;
    }
  }
  return false;
}

void RecheckCollision::checkSingle(TrajectorySegment *segment) {
  // Recursive removal
  int j = 0;
  for (int i = 0; i < segment->children.size(); ++i) {
    if (isCollided(segment->children[j]->trajectory)) {
      segment->children.erase(segment->children.begin() + j);
    } else {
      j++;
    }
  }
  // remaining children
  for (int i = 0; i < segment->children.size(); ++i) {
    checkSingle(segment->children[i].get());
  }
}

} // namespace generator_updater
} // namespace active_3d_planning
