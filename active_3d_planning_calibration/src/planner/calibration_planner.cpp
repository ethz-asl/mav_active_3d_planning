#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "active_3d_planning/planner/calibration_planner.h"

#include <std_srvs/Empty.h>

namespace active_3d_planning {
namespace ros {

CalibrationPlanner::CalibrationPlanner(const ::ros::NodeHandle &nh,
                                       const ::ros::NodeHandle &nh_private)
    : RosPlanner(nh, nh_private) 
    {
      pause_simulation_srv_=nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
      unpause_simulation_srv_=nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
      calibrate_srv_=nh_.serviceClient<std_srvs::Empty>("/calibrate_service");
    }

void CalibrationPlanner::loopIteration() {
  // Continuosly expand the trajectory space
  for (int i = 0; i < p_expand_batch_; ++i) {
    expandTrajectories();
  }

  if (!min_new_value_reached_) {
    //  recursively check whether the minimum value is reached
    min_new_value_reached_ = checkMinNewValue(current_segment_);
  }

  // After finishing the current segment, execute the next one
  if (target_reached_) {
    std_srvs::Empty emptySrv;
    pause_simulation_srv_.call(emptySrv);
    // This is a publisher to inform of target reached
    target_reached_pub_.publish(std_msgs::Empty());
    calibrate_srv_.call(emptySrv);
    unpause_simulation_srv_.call(emptySrv);

    if (new_segment_tries_ >= p_max_new_tries_ && p_max_new_tries_ > 0) {
      // Maximum tries reached: force next segment
      requestNextTrajectory();
    } else {
      if (new_segment_tries_ < p_min_new_tries_) {
        ROS_INFO("Exiting because min tries was not reached!");
        return; // check minimum tries reached
      }
      if (new_segments_ < p_min_new_segments_) {
        ROS_INFO("Exiting because min new segments was not reached!");
        return; // check minimum successful expansions reached
      }
      if (!min_new_value_reached_) {
        ROS_INFO("Exiting because min value was not reached!");
        return; // check minimum value reached
      }
      // All requirements met
      requestNextTrajectory();
    }
  }
}

} // namespace ros
} // namespace active_3d_planning