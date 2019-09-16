#ifndef ACTIVE_3D_PLANNING_CORE_TOOLS_VISUALIZER_H
#define ACTIVE_3D_PLANNING_CORE_TOOLS_VISUALIZER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace active_3d_planning {

struct Color {
  double r = 0;
  double g = 0;
  double b = 0;
  double a = 1;
};
// TODO default values
struct VisualizationMarker {
  // TODO but all necessary information here
  // from ROS msg documentation
  unsigned char ARROW = 0;
  unsigned char CUBE = 1;
  unsigned char SPHERE = 2;
  unsigned char CYLINDER = 3;
  unsigned char LINE_STRIP = 4;
  unsigned char LINE_LIST = 5;
  unsigned char CUBE_LIST = 6;
  unsigned char SPHERE_LIST = 7;
  unsigned char POINTS = 8;
  unsigned char TEXT_VIEW_FACING = 9;
  unsigned char MESH_RESOURCE = 10;
  unsigned char TRIANGLE_LIST = 11;
  unsigned char ADD = 0;
  unsigned char MODIFY = 0;
  unsigned char DELETE = 2;
  unsigned char DELETEALL = 3;
  int type = 0;
  int action = 0;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d scale;
  Color color;
  std::vector<Eigen::Vector3d> points;
  std::vector<Color> colors;
};

class VisualizerI {
  virtual ~VisualizerI() = default;

  virtual void addMarker(const VisualizationMarker &marker) = 0;
};
} // namespace active_3d_planning
#endif /* ACTIVE_3D_PLANNING_CORE_TOOLS_VISUALIZER_H */