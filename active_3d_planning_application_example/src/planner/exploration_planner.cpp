#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "active_3d_planning/planner/exploration_planner.h"

#include <mav_msgs/default_topics.h>

namespace active_3d_planning {
namespace ros {

ExplorationPlanner::ExplorationPlanner(const ::ros::NodeHandle &nh,
                                const ::ros::NodeHandle &nh_private)
    : RosPlanner(nh, nh_private) {
  target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
}

} // namespace ros
} // namespace active_3d_planning