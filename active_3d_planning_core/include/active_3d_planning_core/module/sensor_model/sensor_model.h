#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_SENSOR_MODEL_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_SENSOR_MODEL_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/data/visualization_markers.h"
#include "active_3d_planning_core/map/occupancy_map.h"
#include "active_3d_planning_core/module/module_factory.h"

namespace active_3d_planning {

// Base class that interfaces with sensor based evaluators.
class SensorModel : public Module {
 public:
  explicit SensorModel(PlannerI& planner);  // NOLINT

  virtual ~SensorModel() = default;

  // Return the voxel centers of all visible voxels
  virtual bool getVisibleVoxelsFromTrajectory(
      std::vector<Eigen::Vector3d>* result,
      const TrajectorySegment& traj_in) = 0;

  // Implement this function to allow visualization of the sensing bounds
  virtual void visualizeSensorView(VisualizationMarkers* markers,
                                   const TrajectorySegment& trajectory) = 0;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  // sensor model is configured to use voxel maps
  map::OccupancyMap* map_;

  // mounting transform from body (pose) to sensor , in body frame
  Eigen::Vector3d mounting_translation_;  // x,y,z [m]
  Eigen::Quaterniond mounting_rotation_;  // x,y,z,w quaternion
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_SENSOR_MODEL_H_
