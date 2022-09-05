#include <iostream>
#include <sstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include "active_3d_planning_core/tools/defaults.h"
#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"
#include "active_3d_planning_ros/tools/ros_conversion.h"

namespace active_3d_planning {

class TrajectoryAdapter {
 public:
  TrajectoryAdapter(const ::ros::NodeHandle& nh,
                    const ::ros::NodeHandle& nh_private);

  virtual ~TrajectoryAdapter() {}

  // ros callbacks
  void odometryCallback(const nav_msgs::Odometry& msg);

  // Can use Pose or Trajectory as input. For Trajectory only the end pose is
  // accounted for!
  void commandPoseCallback(const geometry_msgs::Pose& msg);
  void commandTrajectorycallback(
      const trajectory_msgs::MultiDOFJointTrajectory& trajectory);

 protected:
  // ros
  ::ros::NodeHandle nh_;
  ::ros::NodeHandle nh_private_;
  ::ros::Subscriber odom_sub_;
  ::ros::Subscriber pose_sub_;
  ::ros::Subscriber traj_sub_;
  ::ros::Publisher traj_pub_;
  ::ros::Publisher vis_pub_;

  // variables
  Eigen::Vector3d current_position_;  // Current pose of the mav
  Eigen::Vector3d current_v_;
  Eigen::Quaterniond current_orientation_;
  Eigen::Vector3d current_rotation_rate_;
  double current_yaw_;
  Eigen::Vector3d goal_pos_;
  double goal_yaw_;
  bool goal_reached_;
  bool republish_paths_;
  bool check_goal_reached_;
  bool start_at_zero_velocity_;

  // generator
  LinearMavTrajectoryGenerator generator_;
};

TrajectoryAdapter::TrajectoryAdapter(const ::ros::NodeHandle& nh,
                                     const ::ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      goal_reached_(true),
      start_at_zero_velocity_(true),
      republish_paths_(false),
      check_goal_reached_(false) {
  traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  vis_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "/planner/trajectory_visualization", 100);

  odom_sub_ = nh_.subscribe("odometry_in", 1,
                            &TrajectoryAdapter::odometryCallback, this);
  pose_sub_ = nh_.subscribe("command_pose_in", 1,
                            &TrajectoryAdapter::commandPoseCallback, this);
  traj_sub_ =
      nh_.subscribe("command_trajectory_in", 1,
                    &TrajectoryAdapter::commandTrajectorycallback, this);
  float velocity = 1.0;
  float acceleration = 1.0;
  float yaw_rate = 1.57;
  float yaw_acceleration = 1.57;
  float sampling_rate = 20.0;
  nh_private_.param("velocity", velocity, velocity);
  nh_private_.param("acceleration", acceleration, acceleration);
  nh_private_.param("yaw_rate", yaw_rate, yaw_rate);
  nh_private_.param("yaw_acceleration", yaw_acceleration, yaw_acceleration);
  nh_private_.param("sampling_rate", sampling_rate, sampling_rate);
  nh_private_.param("republish_paths", republish_paths_, republish_paths_);
  nh_private_.param("check_goal_reached", check_goal_reached_,
                    check_goal_reached_);
  nh_private_.param("start_at_zero_velocity", start_at_zero_velocity_,
                    start_at_zero_velocity_);
  generator_.setConstraints(
      velocity, acceleration, yaw_rate, yaw_acceleration,
      sampling_rate);  // v, a, yaw_rate, yaw_accel, sampling_rate
}

void TrajectoryAdapter::odometryCallback(const nav_msgs::Odometry& msg) {
  // Track the current pose
  current_position_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  current_v_ =
      Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  current_rotation_rate_ =
      Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                      msg.twist.twist.angular.z);
  current_yaw_ = tf::getYaw(msg.pose.pose.orientation);

  if (check_goal_reached_) {
    if ((goal_pos_ - current_position_).norm() < 0.15 &&
        defaults::angleDifference(goal_yaw_, current_yaw_) < 0.15) {
      goal_reached_ = true;
    }
  }
}

void TrajectoryAdapter::commandTrajectorycallback(
    const trajectory_msgs::MultiDOFJointTrajectory& trajectory) {
  if (trajectory.points.empty()) {
    ROS_WARN("[TrajectoryAdapter] Received empty trajectory, skipping!");
    return;
  }
  geometry_msgs::Pose pose;
  const geometry_msgs::Transform& point =
      trajectory.points.back().transforms.front();
  pose.position.x = point.translation.x;
  pose.position.y = point.translation.y;
  pose.position.z = point.translation.z;
  pose.orientation.x = point.rotation.x;
  pose.orientation.y = point.rotation.y;
  pose.orientation.z = point.rotation.z;
  pose.orientation.w = point.rotation.w;
  commandPoseCallback(pose);
}

void TrajectoryAdapter::commandPoseCallback(const geometry_msgs::Pose& msg) {
  if (!goal_reached_ && check_goal_reached_) {
    return;
  }
  goal_reached_ = false;
  EigenTrajectoryPoint start;
  EigenTrajectoryPoint goal;
  EigenTrajectoryPointVector result;

  // set goal
  start.position_W = current_position_;
  start.orientation_W_B = current_orientation_;
  if (!start_at_zero_velocity_) {
    start.velocity_W = current_v_;
    start.angular_velocity_W = current_rotation_rate_;
  }
  goal.position_W.x() = msg.position.x;
  goal.position_W.y() = msg.position.y;
  goal.position_W.z() = msg.position.z;
  goal.setFromYaw(tf::getYaw(msg.orientation));
  std::cout << "[TrajectoryAdapter]  Optimizing trajectory..." << std::endl;
  if (!generator_.createTrajectory(start, goal, &result)) {
    ROS_WARN(
        "[TrajectoryAdapter] Could not optimize trajectory, using "
        "approximation.");
    generator_.simulateTrajectory(start, goal, &result);
  }

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg_out(
      new trajectory_msgs::MultiDOFJointTrajectory);
  msg_out->header.stamp = ::ros::Time::now();
  const int n_points = result.size();
  // msg_out->points.reserve(n_points);
  std::cout << "[TrajectoryAdapter] Got optimized trajectory of " << n_points
            << " points" << std::endl;
  for (int i = 0; i < n_points; ++i) {
    if (result[i].position_W.norm() <= 1.0e-6) {
      ROS_WARN_STREAM("[TrajectoryAdapter] Found invalid entry at point "
                      << i << ", skipping.");
    } else {
      trajectory_msgs::MultiDOFJointTrajectoryPoint point;
      ros::msgMultiDofJointTrajectoryPointFromEigen(result[i], &point);
      msg_out->points.push_back(point);
    }
  }
  if (msg_out->points.empty()) {
    ROS_WARN("[TrajectoryAdapter] Command is empty, not sending.");
  } else {
    traj_pub_.publish(msg_out);
  }
  goal_pos_ = result.back().position_W;
  goal_yaw_ = result.back().getYaw();
  std::cout << "[TrajectoryAdapter] Sent trajectory." << std::endl;

  // vis
  visualization_msgs::Marker msg_vis;
  msg_vis.header.frame_id = "/world";
  msg_vis.header.stamp = ::ros::Time::now();
  msg_vis.pose.orientation.w = 1.0;
  msg_vis.type = visualization_msgs::Marker::POINTS;
  msg_vis.ns = "completed_trajectory";
  msg_vis.id = 0;
  msg_vis.action = visualization_msgs::Marker::ADD;
  msg_vis.scale.x = 0.1;
  msg_vis.scale.y = 0.1;
  msg_vis.color.r = 1.0;
  msg_vis.color.g = 0.0;
  msg_vis.color.b = 0.0;
  msg_vis.color.a = 1.0;

  // points
  for (int i = 0; i < result.size(); ++i) {
    geometry_msgs::Point point;
    point.x = result[i].position_W[0];
    point.y = result[i].position_W[1];
    point.z = result[i].position_W[2];
    msg_vis.points.push_back(point);
  }
  vis_pub_.publish(msg_vis);
}

}  // namespace active_3d_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_adaptor");

  // Set logging to debug for testing
  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  // ros::console::levels::Debug);

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  active_3d_planning::TrajectoryAdapter adaptor(nh, nh_private);
  ROS_INFO("Initialized TrajectoryAdapter.");
  ros::spin();
}