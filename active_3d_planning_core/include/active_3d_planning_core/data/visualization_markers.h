#ifndef ACTIVE_3D_PLANNING_CORE_DATA_VISUALIZATION_MARKERS_H_
#define ACTIVE_3D_PLANNING_CORE_DATA_VISUALIZATION_MARKERS_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace active_3d_planning {

struct Color {
  double r = 0;
  double g = 0;
  double b = 0;
  double a = 1;
};

struct VisualizationMarker {
  // from ROS visualization_msgs/marker.msg documentation (Only relevant fields)
  const static unsigned char ARROW = 0;
  const static unsigned char CUBE = 1;
  const static unsigned char SPHERE = 2;
  const static unsigned char CYLINDER = 3;
  const static unsigned char LINE_STRIP = 4;
  const static unsigned char LINE_LIST = 5;
  const static unsigned char CUBE_LIST = 6;
  const static unsigned char SPHERE_LIST = 7;
  const static unsigned char POINTS = 8;
  const static unsigned char TEXT_VIEW_FACING = 9;
  const static unsigned char MESH_RESOURCE = 10;
  const static unsigned char TRIANGLE_LIST = 11;
  const static unsigned char ADD = 0;
  const static unsigned char MODIFY = 0;
  const static unsigned char DELETE = 2;
  const static unsigned char DELETEALL = 3;
  const static unsigned char OVERWRITE =
      4;  // added this option as RVIZ needs to remove the remaining markers
  int type = 0;
  int action = 0;
  int id = 0;
  std::string ns = "";
  std::string text = "";
  Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
  Eigen::Quaterniond orientation = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d scale = Eigen::Vector3d(0, 0, 0);
  Color color = Color();
  std::vector<Eigen::Vector3d> points;  // empty initialization
  std::vector<Color> colors;
};

struct VisualizationMarkers {
  // access
  void addMarker(const VisualizationMarker& marker);

  int getNextVisualizationId();

  const std::vector<VisualizationMarker>& getMarkers() const;

  std::vector<VisualizationMarker>& getMarkers();

 protected:
  std::vector<VisualizationMarker> markers;
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_DATA_VISUALIZATION_MARKERS_H_
