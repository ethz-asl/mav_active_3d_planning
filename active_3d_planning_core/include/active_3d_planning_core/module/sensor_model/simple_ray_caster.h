#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_SIMPLE_RAY_CASTER_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_SIMPLE_RAY_CASTER_H_

#include <vector>

#include "active_3d_planning_core/data/trajectory_segment.h"
#include "active_3d_planning_core/module/sensor_model/camera_model.h"
#include "active_3d_planning_core/tools/defaults.h"

namespace active_3d_planning {
namespace sensor_model {

// Get minimum resolution reuqired for 1 ray per voxel at max distance, then
// cast a ray through every such voxel to detect visible voxels (timed at 98 +/-
// 30 ms)
class SimpleRayCaster : public CameraModel {
 public:
  explicit SimpleRayCaster(PlannerI& planner);  // NOLINT

  virtual ~SimpleRayCaster() = default;

  // Override virutal functions
  bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                        const Eigen::Vector3d& position,
                        const Eigen::Quaterniond& orientation) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<SimpleRayCaster> registration;

  // params
  double p_ray_step_;
  double p_downsampling_factor_;  // Artificially reduce the minimum resolution
  // to increase performance

  // constants
  int c_res_x_;  // factual resolution that is used for ray casting
  int c_res_y_;
};

}  // namespace sensor_model
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_SIMPLE_RAY_CASTER_H_
