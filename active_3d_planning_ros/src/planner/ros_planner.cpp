#include "active_3d_planning_ros/planner/ros_planner.h"

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <mav_msgs/conversions.h>

#include "active_3d_planning_ros/module/module_factory_ros.h"
#include "active_3d_planning_ros/tools/ros_conversion.h"

namespace active_3d_planning {
namespace ros {

RosPlanner::RosPlanner(const ::ros::NodeHandle& nh,
                       const ::ros::NodeHandle& nh_private,
                       ModuleFactory* factory, Module::ParamMap* param_map)
    : OnlinePlanner(factory, param_map), nh_(nh), nh_private_(nh_private) {
  // params
  RosPlanner::setupFromParamMap(param_map);
  perf_log_data_ = std::vector<double>(
      6, 0.0);  // select, expand, gain, cost, value, mainLoop, rosCallbacks

  // Subscribers and publishers
  target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "command/trajectory", 10);

  // Publishing the Eigen trajectory before conversion to MultiDOF
  target_pub_path_ = nh_.advertise<nav_msgs::Path>(
      "command/trajectory_path", 10);
    
  trajectory_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "trajectory_visualization", 100);
  odom_sub_ = nh_.subscribe("odometry", 1, &RosPlanner::odomCallback, this);
  get_cpu_time_srv_ = nh_private_.advertiseService(
      "get_cpu_time", &RosPlanner::cpuSrvCallback, this);

  // Iterate through goal_topics delimited by ',' and subscribe to each
  if (p_goal_topics_.empty()) {
    ROS_ERROR("No goal topics specified!");
  } else {
    std::stringstream ss(p_goal_topics_);
    std::string topic;
    while (std::getline(ss, topic, ',')) {
      goal_subs_.push_back(nh_.subscribe(topic, 1, &RosPlanner::goalCallback,
                                        this));
    }
  }

  // Finish
  ROS_INFO_STREAM(
      "\n******************** Initialized Planner ********************\n"
      "Initialized 'RosPlanner' from namespace '"
      << param_map->at("param_namespace")
      << "' with parameters:" << param_map->at("verbose_text"));
}

void RosPlanner::setupFromParamMap(Module::ParamMap* param_map) {
  setParam<double>(param_map, "replan_pos_threshold", &p_replan_pos_threshold_,
                   0.1);
  setParam<double>(param_map, "replan_yaw_threshold", &p_replan_yaw_threshold_,
                   0.1);

  // Param of list of topics
  setParam<std::string>(param_map, "goal_topics", &p_goal_topics_,
                                     std::string());
}

void RosPlanner::setupFactoryAndParams(ModuleFactory* factory,
                                       Module::ParamMap* param_map,
                                       const ::ros::NodeHandle& nh_private) {
  // setup module factory
  factory = dynamic_cast<ModuleFactoryROS*>(factory);
  if (!factory) {
    delete factory;
    factory = new ModuleFactoryROS();
  }

  // setup params
  *param_map = Module::ParamMap();
  std::string type;
  const std::string& args = nh_private.getNamespace();
  factory->getParamMapAndType(param_map, &type, args);
  param_map->at("verbose_text") = "";
}

void RosPlanner::initializePlanning() {
  // setup standard
  OnlinePlanner::initializePlanning();

  // Update performance log for simulated time and ros cpu consumption
  if (p_log_performance_) {
    perf_cpu_timer_ = std::clock();
    perf_log_file_ << ",RosTime,RosCallbacks";
  }

  // Setup counters
  cpu_srv_timer_ = std::clock();
  ros_timing_ = ::ros::Time::now();
  perf_log_data_[5] = 0;  // reset count
}

void RosPlanner::planningLoop() {
  // This is the main loop, spinning is managed explicitely for efficiency
  ROS_INFO(
      "\n******************** Planner is now Running ********************\n");
  run_srv_ = nh_private_.advertiseService("toggle_running",
                                          &RosPlanner::runSrvCallback, this);
  running_ = true;
  std::clock_t timer;
  while (::ros::ok()) {
    if (planning_) {
      loopIteration();
    }
    if (p_log_performance_) {
      timer = std::clock();
    }
    ::ros::spinOnce();
    if (p_log_performance_) {
      perf_log_data_[5] +=
          static_cast<double>(std::clock() - timer) / CLOCKS_PER_SEC;
    }
  }
}

bool RosPlanner::requestNextTrajectory() {
  // call standard procedure
  if (!OnlinePlanner::requestNextTrajectory()) {
    return false;
  }

  // Performance log
  if (p_log_performance_) {
    perf_log_file_ << "," << (::ros::Time::now() - ros_timing_).toSec() << ","
                   << perf_log_data_[5];
    perf_cpu_timer_ = std::clock();
    ros_timing_ = ::ros::Time::now();
    perf_log_data_[5] = 0;  // reset count
  }
  return true;
}

void RosPlanner::odomCallback(const nav_msgs::Odometry& msg) {
  // Track the current pose
  current_position_ =
      Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                      msg.pose.pose.position.z);
  current_orientation_ = Eigen::Quaterniond(
      msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
  if (running_ && !target_reached_) {
    // check goal pos reached (if tol is set)
    if (p_replan_pos_threshold_ <= 0 ||
        (target_position_ - current_position_).norm() <
            p_replan_pos_threshold_) {
      // check goal yaw reached (if tol is set)
      double yaw = tf::getYaw(msg.pose.pose.orientation);
      if (p_replan_yaw_threshold_ <= 0 ||
          defaults::angleDifference(target_yaw_, yaw) <
              p_replan_yaw_threshold_) {
        target_reached_ = true;
      }
    }
  }
}

void RosPlanner::goalCallback(const geometry_msgs::TransformStamped& msg) {
  // Get child frame id
  const std::string& child_frame_id = msg.child_frame_id;
  if (child_frame_id.empty()) {
    LOG(WARNING) << "Received transform without child frame id";
    return;
  }

  // Extract pose from TransformStamped into a Vector3d
  Eigen::Vector3d pose;
  pose.x() = msg.transform.translation.x;
  pose.y() = msg.transform.translation.y;
  pose.z() = msg.transform.translation.z;

  // Update trajectory generator, pass by reference
  trajectory_generator_->updateGoals(child_frame_id, pose);
}

void RosPlanner::requestMovement(const EigenTrajectoryPointVector& trajectory) {
  if (trajectory.empty()) {
    LOG(WARNING) << "Tried to publish an empty trajectory";
    return;
  }

  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(
      new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ::ros::Time::now();

  nav_msgs::Path msg_path;
  msg_path.header.stamp = ::ros::Time::now();
  msg_path.header.frame_id = "world";

  int n_points = trajectory.size();
  msg->points.resize(n_points);
  msg_path.poses.resize(n_points);

  for (int i = 0; i < n_points; ++i) {
    msgMultiDofJointTrajectoryPointFromEigen(trajectory[i], &msg->points[i]);
  
    // Publish Path msg
    const mav_msgs::EigenTrajectoryPoint point_mav(trajectory[i].time_from_start_ns,
                                               trajectory[i].position_W,
                                               trajectory[i].velocity_W,
                                               trajectory[i].acceleration_W,
                                               trajectory[i].jerk_W,
                                               trajectory[i].snap_W,
                                               trajectory[i].orientation_W_B,
                                               trajectory[i].angular_velocity_W,
                                               trajectory[i].angular_acceleration_W);
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(point_mav, &msg_path.poses[i]);
  }

  target_pub_.publish(msg);
  target_pub_path_.publish(msg_path);
}

void RosPlanner::publishVisualization(const VisualizationMarkers& markers) {
  if (markers.getMarkers().empty()) {
    return;
  }
  visualization_msgs::MarkerArray msg;
  visualizationMarkersToMsg(markers, &msg);
  for (visualization_msgs::Marker& m : msg.markers) {
    m.header.frame_id = "world";
    m.header.stamp = ::ros::Time::now();
  }
  // check overwrite flag (for full array)
  if (markers.getMarkers()[0].action == VisualizationMarker::OVERWRITE) {
    for (visualization_msgs::Marker& m : msg.markers) {
      m.action = visualization_msgs::Marker::ADD;
    }
    std::map<std::string, int>::iterator it =
        visualization_overwrite_counter_.find(markers.getMarkers()[0].ns);
    int count = 0;
    if (it != visualization_overwrite_counter_.end()) {
      count = it->second;
    }
    if (count > markers.getMarkers().size()) {
      visualization_msgs::Marker marker;
      for (int i = markers.getMarkers().size(); i < count; ++i) {
        // publish empty marker to remove previous ids
        auto empty_marker = visualization_msgs::Marker();
        empty_marker.header.frame_id = "world";
        empty_marker.header.stamp = ::ros::Time::now();
        empty_marker.type = visualization_msgs::Marker::POINTS;
        empty_marker.id = i;
        empty_marker.ns = markers.getMarkers()[0].ns;
        empty_marker.action = visualization_msgs::Marker::DELETE;
        msg.markers.push_back(empty_marker);
      }
    }
    visualization_overwrite_counter_[markers.getMarkers()[0].ns] =
        markers.getMarkers().size();
  }
  trajectory_vis_pub_.publish(msg);
}

bool RosPlanner::runSrvCallback(std_srvs::SetBool::Request& req,
                                std_srvs::SetBool::Response& res) {
  res.success = true;
  if (req.data) {
    initializePlanning();
    planning_ = true;
    ROS_INFO("Started planning.");
  } else {
    planning_ = false;
    ROS_INFO("Stopped planning.");
  }
  return true;
}

bool RosPlanner::cpuSrvCallback(std_srvs::SetBool::Request& req,
                                std_srvs::SetBool::Response& res) {
  double time =
      static_cast<double>(std::clock() - cpu_srv_timer_) / CLOCKS_PER_SEC;
  cpu_srv_timer_ = std::clock();

  // Just return cpu time as the service message
  res.message = std::to_string(time).c_str();
  res.success = true;
  return true;
}

// logging and printing
void RosPlanner::printInfo(const std::string& text) { ROS_INFO_STREAM(text); }

void RosPlanner::printWarning(const std::string& text) {
  ROS_WARN_STREAM(text);
}

void RosPlanner::printError(const std::string& text) { ROS_ERROR_STREAM(text); }

}  // namespace ros
}  // namespace active_3d_planning
