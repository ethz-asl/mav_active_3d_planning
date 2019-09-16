#include "active_3d_planning/tools/ros_visualizer.h"

#include "active_3d_planning/tools/ros_conversion.h"

namespace active_3d_planning {
namespace ros {

// Visualization message
int getNextVisualizationId(const visualization_msgs::MarkerArray &msg) {
  if (msg.markers.empty()) {
    return 0;
  } else {
    return msg.markers.back().id + 1;
  }
}

void RosVisualizer::addMarker(const VisualizationMarker &marker) {
  // Build message
  visualization_msgs::Marker new_msg;
  new_msg.ns = "evaluation";
  new_msg.header.stamp = ros::Time::now();
  new_msg.header.frame_id = "/world";
  new_msg.id = getNextVisualizationId(msgEval_); 
  
  new_msg.type = marker.type;
  new_msg.action = marker.action;

  vectorEigenToMsg(marker.position, new_msg.pose.position);
  quaternionEigenToMsg(marker.orientation, new_msg.pose.orientation);
  vectorEigenToMsg(marker.scale, new_msg.scale);

  new_msg.color.r = marker.color.r;
  new_msg.color.g = marker.color.g;
  new_msg.color.b = marker.color.b;
  new_msg.color.a = marker.color.a;

  for(const auto& point : marker.points){
    geometry_msgs::Point rospoint;
    vectorEigenToMsg(point, rospoint);
    new_msg.points.push_back(rospoint);
  }

  for(const auto& color : marker.colors){
    geometry_msgs::Point roscolor;
    roscolor.r = color.r;
    roscolor.g = color.g;
    roscolor.b = color.b;
    roscolor.a = color.a;
    new_msg.colors.push_back(roscolor);
  }

  msgEval_.markers.push_back(new_msg);
}

void RosVisualizer::publishEvalVisualization(
    const TrajectorySegment &trajectory) {
  // Visualize the gain of the current segment
  msgEval_ = visualization_msgs::MarkerArray();
  trajectory_evaluator_->visualizeTrajectoryValue(*this, trajectory);
  if (msgEval_.markers.size() < vis_num_previous_evaluations_) {
    int temp_num_evals = msgEval_.markers.size();
    // Make sure previous visualization is cleared again
    for (int i = msgEval_.markers.size(); i < vis_num_previous_evaluations_;
         ++i) {
      // Setup marker message
      visualization_msgs::Marker new_msg;
      new_msg.header.frame_id = "/world";
      new_msg.header.stamp = ::ros::Time::now();
      new_msg.id = i;
      new_msg.ns = "evaluation";
      new_msg.action = visualization_msgs::Marker::DELETE;
      msgEval_.markers.push_back(new_msg);
    }
    vis_num_previous_evaluations_ = temp_num_evals;
  }
  trajectory_vis_pub_.publish(msgEval_);
}
} // namespace ros
} // namespace active_3d_planning