#include "active_3d_planning_mav/tools/linear_mav_trajectory_generator.h"
#include "active_3d_planning_core/tools/defaults.h"
#include "active_3d_planning_ros/tools/ros_conversion.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <vector>



namespace active_3d_planning {

class TrajectoryAdaptor {
 public:
  TrajectoryAdaptor(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private);

  virtual ~TrajectoryAdaptor() {}

  // ros callbacks
  void odometryCallback(const nav_msgs::Odometry &msg);
  void requestCallback(const geometry_msgs::Pose &msg);
 protected:
  // ros
  ::ros::NodeHandle nh_;
  ::ros::NodeHandle nh_private_;
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
  int vis_completed_count_;
  bool republish_paths_;
  bool check_goal_reached_;

  // generator
  LinearMavTrajectoryGenerator generator_;
};

TrajectoryAdaptor::TrajectoryAdaptor(const ::ros::NodeHandle &nh, const ::ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      goal_reached_(true),
      vis_completed_count_(0),
      republish_paths_(false),
      check_goal_reached_(false) {
  traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/trajectory_visualization", 100);

  pose_sub_ = nh_.subscribe("odometry_in", 1, &TrajectoryAdaptor::odometryCallback, this);
  traj_sub_ = nh_.subscribe("request_in", 1, &TrajectoryAdaptor::requestCallback, this);
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
  nh_private_.param("check_goal_reached", check_goal_reached_, check_goal_reached_);
  generator_.setConstraints(velocity, acceleration, yaw_rate, yaw_acceleration, sampling_rate);  // v, a, yaw_rate, yaw_accel, sampling_rate
}

void TrajectoryAdaptor::odometryCallback(const nav_msgs::Odometry &msg) {
  // Track the current pose
  current_position_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                      msg.pose.pose.position.z);
  current_v_ = Eigen::Vector3d(msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                               msg.twist.twist.linear.z);
  current_orientation_ = Eigen::Quaterniond(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w);
  current_rotation_rate_ = Eigen::Vector3d(msg.twist.twist.angular.x, msg.twist.twist.angular.y,
                               msg.twist.twist.angular.z);
  current_yaw_ = tf::getYaw(msg.pose.pose.orientation);

  if (check_goal_reached_) {
    if ((goal_pos_ - current_position_).norm() < 0.15
        && defaults::angleDifference(goal_yaw_, current_yaw_) < 0.15) {
      goal_reached_ = true;
    }
  }
}

void TrajectoryAdaptor::requestCallback(const geometry_msgs::Pose &msg) {
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
  start.velocity_W = current_v_;
  start.angular_velocity_W = current_rotation_rate_;
  goal.position_W.x() = msg.position.x;
  goal.position_W.y() = msg.position.y;
  goal.position_W.z() = msg.position.z;
  goal.setFromYaw(tf::getYaw(msg.orientation));

  if (!generator_.createTrajectory(start, goal, &result)) {
    generator_.simulateTrajectory(start, goal, &result);
  }

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg_out(new trajectory_msgs::MultiDOFJointTrajectory);
  msg_out->header.stamp = ::ros::Time::now();
  int n_points = result.size();
  msg_out->points.resize(n_points);
  for (int i = 0; i < n_points; ++i) {
    ros::msgMultiDofJointTrajectoryPointFromEigen(result[i], &(msg_out->points[i]));
  }
  traj_pub_.publish(msg_out);
  goal_pos_ = result.back().position_W;
  goal_yaw_ = result.back().getYaw();

  // vis
  visualization_msgs::Marker msg_vis;
  msg_vis.header.frame_id = "/world";
  msg_vis.header.stamp = ::ros::Time::now();
  msg_vis.pose.orientation.w = 1.0;
  msg_vis.type = visualization_msgs::Marker::POINTS;
  msg_vis.ns = "completed_trajectory";
  msg_vis.id = vis_completed_count_;
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
  visualization_msgs::MarkerArray array_msg;
  array_msg.markers.push_back(msg_vis);
  vis_pub_.publish(array_msg);
  vis_completed_count_++;
}

} // namespace active_3d_planning


int main(int argc, char **argv) {
  ros::init(argc, argv, "trajectory_adaptor");

  // Set logging to debug for testing
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  active_3d_planning::TrajectoryAdaptor adaptor(nh, nh_private);
  ROS_INFO("Initialized TrajectoryAdaptor.");
  ros::spin();
}