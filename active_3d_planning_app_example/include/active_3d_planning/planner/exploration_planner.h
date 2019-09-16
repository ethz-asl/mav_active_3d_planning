#ifndef MAV_ACTIVE_3D_PLANNING_EXPLORATION_EXPLORATION_PLANNER_H_
#define MAV_ACTIVE_3D_PLANNING_EXPLORATION_EXPLORATION_PLANNER_H_

#include "active_3d_planning/planner/ros_planner.h"

namespace active_3d_planning {
namespace ros {

class ExplorationPlanner : public RosPlanner {
public:
  ExplorationPlanner(const ::ros::NodeHandle &nh,
                     const ::ros::NodeHandle &nh_private);

  virtual ~ExplorationPlanner() = default;
};
} // namespace ros
} // namespace active_3d_planning

#endif // MAV_ACTIVE_3D_PLANNING_EXPLORATION_EXPLORATION_PLANNER_H_
