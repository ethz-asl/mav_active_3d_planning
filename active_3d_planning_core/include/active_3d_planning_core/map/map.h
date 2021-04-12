#ifndef ACTIVE_3D_PLANNING_CORE_MAP_MAP_H_
#define ACTIVE_3D_PLANNING_CORE_MAP_MAP_H_

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/module/module.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {

// base interface for the bare minimum any map must provide
class Map : public Module {
 public:
  virtual ~Map() = default;

  explicit Map(PlannerI& planner);  // NOLINT

  // check collision for a single pose
  virtual bool isTraversable(const Eigen::Vector3d& position,
                             const Eigen::Quaterniond& orientation =
                                 Eigen::Quaterniond(1, 0, 0, 0)) = 0;

  // check collision for a path
  virtual bool isTraversablePath(const EigenTrajectoryPointVector& trajectory);

  // check whether point is part of the map
  virtual bool isObserved(const Eigen::Vector3d& point) = 0;
};

}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MAP_MAP_H_
