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

bool RecheckCollision::updateSegment(TrajectorySegment *segment) {
    for (int i = 0; i < segment->trajectory.size(); ++i) {
        if (!(planner_.getTrajectoryGenerator().checkTraversable(
                segment->trajectory[i].position_W))) {
            return false;
        }
    }
    return true;
}

} // namespace generator_updater
} // namespace active_3d_planning