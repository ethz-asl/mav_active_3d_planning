#ifndef ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_CONVERSION_H_
#define ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_CONVERSION_H_

#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

#include "active_3d_planning_core/data/visualization_markers.h"

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
    const EigenTrajectoryPoint& trajectory_point,
    trajectory_msgs::MultiDOFJointTrajectoryPoint* msg) {
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

inline void visualizationMarkerToMsg(const VisualizationMarker& marker,
                                     visualization_msgs::Marker* msg) {
  assert(msg != NULL);
  msg->type = marker.type;
  msg->action = marker.action;
  msg->id = marker.id;
  msg->ns = marker.ns;
  msg->text = marker.text;
  msg->pose.position.x = marker.position.x();
  msg->pose.position.y = marker.position.y();
  msg->pose.position.z = marker.position.z();
  msg->pose.orientation.x = marker.orientation.x();
  msg->pose.orientation.y = marker.orientation.y();
  msg->pose.orientation.z = marker.orientation.z();
  msg->pose.orientation.w = marker.orientation.w();
  msg->scale.x = marker.scale.x();
  msg->scale.y = marker.scale.y();
  msg->scale.z = marker.scale.z();
  msg->color.r = marker.color.r;
  msg->color.g = marker.color.g;
  msg->color.b = marker.color.b;
  msg->color.a = marker.color.a;
  msg->points.clear();
  geometry_msgs::Point point;
  for (const Eigen::Vector3d& p : marker.points) {
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    msg->points.push_back(point);
  }
  msg->colors.clear();
  std_msgs::ColorRGBA color;
  for (const Color& c : marker.colors) {
    color.r = c.r;
    color.g = c.g;
    color.b = c.b;
    color.a = c.a;
    msg->colors.push_back(color);
  }
}

inline void visualizationMarkersToMsg(const VisualizationMarkers& markers,
                                      visualization_msgs::MarkerArray* msg) {
  assert(msg != NULL);
  visualization_msgs::Marker marker;
  msg->markers.clear();
  for (const VisualizationMarker& m : markers.getMarkers()) {
    visualizationMarkerToMsg(m, &marker);
    msg->markers.push_back(marker);
  }
}

}  // namespace ros
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_CONVERSION_H_
