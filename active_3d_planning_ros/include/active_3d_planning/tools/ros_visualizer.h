#ifndef ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_VISUALIZER_H
#define ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_VISUALIZER_H

#include "active_3d_planning/tools/visualizer_I.h"

#include <visualization_msgs/MarkerArray.h>

namespace active_3d_planning {
namespace ros {

// Get the current next id for visualization markerarray messages
int getNextVisualizationId(const visualization_msgs::MarkerArray &msg);

class RosVisualizer : public VisualizerI {
public:
  RosVisualizer() = default;
  virtual ~RosVisualizer() = default;

  virtual void addMarker(const VisualizationMarker &marker) override;

  void publishEvalVisualization(const TrajectorySegment &trajectory);

private:
  visualization_msgs::MarkerArray msgEval_;
};
} // namespace ros
} // namespace active_3d_planning
#endif /* ACTIVE_3D_PLANNING_ROS_TOOLS_ROS_VISUALIZER_H */