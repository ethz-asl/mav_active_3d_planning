#ifndef ACTIVE_3D_PLANNING_PANOPTIC_MAP_PANOPTIC_H_
#define ACTIVE_3D_PLANNING_PANOPTIC_MAP_PANOPTIC_H_

#include <memory>

#include <active_3d_planning_core/module/module_factory_registry.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping_ros/panoptic_mapper.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "active_3d_planning_core/map/voxel_value_map.h"

#include "panoptic_mapping/tools/planning_interface.h"

namespace active_3d_planning {
namespace map {

// panoptic as a map representation
class PanopticMap : public VoxelValueMap {
  // Which value should be used as value from the map
  enum ValueType { UNCERTAINTY, VOXEL_ENTROPY, VOXEL_PROBABILITY };

 public:
  explicit PanopticMap(PlannerI& planner);  // NOLINT

  // implement virtual methods
  void setupFromParamMap(Module::ParamMap* param_map) override;

  // check collision for a single pose
  bool isTraversable(const Eigen::Vector3d& position,
                     const Eigen::Quaterniond& orientation) override;

  // check whether point is part of the map
  bool isObserved(const Eigen::Vector3d& point) override;

  // get occupancy
  unsigned char getVoxelState(const Eigen::Vector3d& point) override;

  // get voxel size
  double getVoxelSize() override;

  // get the center of a voxel from input point
  bool getVoxelCenter(Eigen::Vector3d* center,
                      const Eigen::Vector3d& point) override;

  // get the stored distance
  double getVoxelDistance(const Eigen::Vector3d& point) override;

  // get the stored weight
  double getVoxelWeight(const Eigen::Vector3d& point) override;

  // get the maximum allowed weight (return 0 if using uncapped weights)
  double getMaximumWeight() override;

  double getVoxelValue(const Eigen::Vector3d& point) override;

  unsigned char getValueVoxelState(const Eigen::Vector3d& matrix) override;

  int getVoxelObservedCount(const Eigen::Vector3d& point) override;

  // Updates the internal panoptic mapper that is used
  void setPanopticMapper(
      std::shared_ptr<panoptic_mapping::PanopticMapper> mapper) {
    mapper_ = mapper;
  }

 private:
  static ModuleFactoryRegistry::Registration<PanopticMap> registration;
  static constexpr float kObservedMinWeight_ = 1e-6;
  // Default Value type
  ValueType value_type = UNCERTAINTY;
  std::shared_ptr<panoptic_mapping::PanopticMapper> mapper_;
  bool getClassVoxelAt(const Eigen::Vector3d& point,
                       panoptic_mapping::ClassVoxelType& voxel);
  const panoptic_mapping::Submap* getPlanningSubmap();
};

}  // namespace map
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_PANOPTIC_MAP_PANOPTIC_H_
