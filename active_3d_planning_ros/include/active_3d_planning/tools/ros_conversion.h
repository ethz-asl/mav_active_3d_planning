#ifndef ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_CONVERSION
#define ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_CONVERSION

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <Eigen/Core>

namespace active_3d_planning {
namespace ros {

inline void vectorEigenToMsg(const Eigen::Vector3d& eigen,
                             geometry_msgs::Vector3* msg) {
  assert(msg != NULL);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
}

inline void quaternionEigenToMsg(const Eigen::Quaterniond& eigen,
                                 geometry_msgs::Quaternion* msg) {
  assert(msg != NULL);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
  msg->w = eigen.w();
}

inline void msgMultiDofJointTrajectoryPointFromEigen(
    const EigenTrajectoryPoint &trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectoryPoint *msg) {
  assert(msg != NULL);

  msg->time_from_start.fromNSec(trajectory_point.time_from_start_ns);
  msg->transforms.resize(1);
  msg->velocities.resize(1);
  msg->accelerations.resize(1);

  vectorEigenToMsg(trajectory_point.position_W,
                   &msg->transforms[0].translation);
  quaternionEigenToMsg(trajectory_point.orientation_W_B,
                       &msg->transforms[0].rotation);
  vectorEigenToMsg(trajectory_point.velocity_W, &msg->velocities[0].linear);
  vectorEigenToMsg(trajectory_point.angular_velocity_W,
                   &msg->velocities[0].angular);
  vectorEigenToMsg(trajectory_point.acceleration_W,
                   &msg->accelerations[0].linear);
}
} // namespace ros
} // namespace active_3d_planning
#endif /* ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_CONVERSION */