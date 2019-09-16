#ifndef ACTIVE_3D_PLANNING_SENSOR_MODEL_SENSOR_MODEL_H
#define ACTIVE_3D_PLANNING_SENSOR_MODEL_SENSOR_MODEL_H

#include "active_3d_planning/module/module_factory.h"
#include "active_3d_planning/data/trajectory_segment.h"

#include <Eigen/Core>

#include <memory>
#include <string>

namespace voxblox {
class EsdfMap;
}

namespace active_3d_planning {

class VisualizerI;

// Base class that interfaces with sensor based evaluators.
class SensorModel : public Module {
public:
  SensorModel(PlannerI &planner);
  virtual ~SensorModel() = default;

  // Return the voxel centers of all visible voxels
  virtual bool
  getVisibleVoxelsFromTrajectory(std::vector<Eigen::Vector3d> *result,
                                 const TrajectorySegment &traj_in) = 0;

  // Implement this function to allow visualization of the sensing bounds
  virtual void visualizeSensorView(VisualizerI& visualizer,
                                   const TrajectorySegment &trajectory) = 0;

  virtual void setupFromParamMap(Module::ParamMap *param_map) override;

protected:
  // voxblox map
  voxblox::EsdfMap &voxblox_;

  // constants
  double c_voxel_size_;
  double c_block_size_;

  // mounting transform from body (pose) to sensor , in body frame
  Eigen::Vector3d mounting_translation_; // x,y,z [m]
  Eigen::Quaterniond mounting_rotation_; // x,y,z,w quaternion

  // changes the input point to the center of the corresponding voxel
  bool getVoxelCenter(Eigen::Vector3d *point);
};

} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_SENSOR_MODEL_SENSOR_MODEL_H
