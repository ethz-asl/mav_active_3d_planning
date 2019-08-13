#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "active_3d_planning/planner/calibration_planner.h"

namespace active_3d_planning {
namespace ros {

CalibrationPlanner::CalibrationPlanner(const ::ros::NodeHandle &nh,
                                       const ::ros::NodeHandle &nh_private)
    : RosPlanner(nh, nh_private) {}

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
    //TODO this somehow has to block until calibration is complete
    //ideally there is some pause functionality
    /* This is a publisher to inform of target reached  */
    target_reached_pub_.publish(std_msgs::Empty());

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