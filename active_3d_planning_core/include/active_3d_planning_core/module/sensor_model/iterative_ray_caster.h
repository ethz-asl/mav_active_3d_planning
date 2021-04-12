#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_ITERATIVE_RAY_CASTER_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_ITERATIVE_RAY_CASTER_H_

#include <vector>

#include "active_3d_planning_core/module/sensor_model/camera_model.h"

namespace active_3d_planning {
namespace sensor_model {

// Cast Rays until they span 1 full voxel, then duplicate them to omit
// redundancies (timed at 42 +/- 20 ms)
class IterativeRayCaster : public CameraModel {
 public:
  explicit IterativeRayCaster(PlannerI& planner);  // NOLINT

  virtual ~IterativeRayCaster() = default;

  // Override virtual functions
  bool getVisibleVoxels(std::vector<Eigen::Vector3d>* result,
                        const Eigen::Vector3d& position,
                        const Eigen::Quaterniond& orientation) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

 protected:
  static ModuleFactoryRegistry::Registration<IterativeRayCaster> registration;

  // params
  double p_ray_step_;
  double p_downsampling_factor_;  // Artificially reduce the minimum resolution
  // to increase performance

  // constants
  int c_res_x_;  // factual resolution that is used for ray casting
  int c_res_y_;
  int c_n_sections_;  // number of ray duplications
  std::vector<double>
      c_split_distances_;            // distances where rays are duplicated
  std::vector<int> c_split_widths_;  // number of max distance rays that are
                                     // covered per split

  // variables
  Eigen::ArrayXXi ray_table_;

  // methods
  void markNeighboringRays(int x, int y, int segment, int value);
};

}  // namespace sensor_model
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_SENSOR_MODEL_ITERATIVE_RAY_CASTER_H_
