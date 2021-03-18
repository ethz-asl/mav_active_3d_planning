#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_LIDAR_MODEL_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_LIDAR_MODEL_H_

#include <string>
#include <vector>

#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/data/visualization_markers.h"
#include "active_3d_planning_core/module/sensor_model/sensor_model.h"

namespace active_3d_planning {
namespace sensor_model {

// Base class for camera models / Utility class that finds visible voxels.
// Available for all trajectory generators. (improve performance here.)
class LidarModel : public SensorModel {
 public:
  explicit LidarModel(PlannerI& planner);  // NOLINT

  virtual ~LidarModel() = default;

  // Return the voxel centers of all visible voxels for a camera model pointing
  // in x-direction
  virtual bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                                const Eigen::Vector3d& position,
                                const Eigen::Quaterniond& orientation) = 0;

  // Return the voxel centers of all visible voxels, sampling camera poses from
  // a trajectory segment
  bool getVisibleVoxelsFromTrajectory(
      std::vector<Eigen::Vector3d>* result,
      const TrajectorySegment& traj_in) override;

  // Display camera view bounds
  void visualizeSensorView(VisualizationMarkers* markers,
                           const TrajectorySegment& traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  // parameters
  double p_ray_length_;  // params for camera model
  double p_fov_x_;  // Total fields of view [deg], expected symmetric w.r.t.
  // sensor facing direction
  double p_fov_y_;
  int p_resolution_x_;
  int p_resolution_y_;
  double p_sampling_time_;  // sample camera poses from segment, use 0 for last
                            // only

  // For performance timing of different implementations
  bool p_test_;
  std::vector<double> time_count_;

  // methods
  // return the indices of the trajectory points of traj_in for which a view is
  // needed
  void sampleViewpoints(std::vector<int>* result,
                        const TrajectorySegment& traj_in);

  void visualizeSingleView(VisualizationMarkers* markers,
                           const Eigen::Vector3d& position,
                           const Eigen::Quaterniond& orientation);

  // get the direction vector for camera pointing in x_direction at pixel with
  // relative x, y position [0, 1]
  void getDirectionVector(Eigen::Vector3d* result, double relative_x,
                          double relative_y);
};

}  // namespace sensor_model
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_LIDAR_MODEL_H_
