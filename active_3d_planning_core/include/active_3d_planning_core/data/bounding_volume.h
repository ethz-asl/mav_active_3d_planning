#ifndef ACTIVE_3D_PLANNING_CORE_DATA_BOUNDING_VOLUME_H_
#define ACTIVE_3D_PLANNING_CORE_DATA_BOUNDING_VOLUME_H_

#include <string>

#include <Eigen/Core>
#include <active_3d_planning_core/data/trajectory_segment.h>
#include <active_3d_planning_core/module/module_factory_registry.h>

namespace active_3d_planning {

// struct for bounding volumes (simple box atm, might include z-rotation,
// sphere, set of planes, ...)
struct BoundingVolume : public Module {
  explicit BoundingVolume(PlannerI& planner);  // NOLINT

  virtual ~BoundingVolume() = default;

  // factory parametrization
  void setupFromFactory(std::string args, bool verbose);

  // populate the bounding volume
  void setupFromParamMap(Module::ParamMap* param_map);

  // check wether point is in bounding box, if bounding box is setup
  bool contains(const Eigen::Vector3d& point);

  // variables
  bool is_setup;
  double x_min, x_max, y_min, y_max, z_min, z_max,
      rotation;  // meters, rotation around Z in deg
  Eigen::Quaterniond rotation_quat;

 protected:
  static ModuleFactoryRegistry::Registration<BoundingVolume> registration;
};

}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_DATA_BOUNDING_VOLUME_H_
