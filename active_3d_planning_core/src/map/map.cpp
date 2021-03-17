#include "active_3d_planning_core/map/map.h"

namespace active_3d_planning {

Map::Map(PlannerI& planner) : Module(planner) {}

bool Map::isTraversablePath(const EigenTrajectoryPointVector& trajectory) {
  // default just checks every point
  for (const EigenTrajectoryPoint& point : trajectory) {
    if (!isTraversable(point.position_W, point.orientation_W_B)) {
      return false;
    }
  }
  return true;
}

}  // namespace active_3d_planning
