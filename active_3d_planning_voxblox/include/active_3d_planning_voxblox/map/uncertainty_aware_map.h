#ifndef ACTIVE_3D_PLANNING_APP_CURIOSITY_UNCERTAINTY_AWARE_MAP_H
#define ACTIVE_3D_PLANNING_APP_CURIOSITY_UNCERTAINTY_AWARE_MAP_H

#include <memory>

#include <active_3d_planning_core/module/module_factory_registry.h>
#include <voxblox_ros/esdf_server.h>

#include "active_3d_planning_core/map/voxel_value_map.h"

#include "voxblox.h"

namespace active_3d_planning {
namespace map {

// Map that consists of two voxblox maps. One is used for reconstruction whereas
// the other is used to store an uncertainty value for each voxel
class UncertaintyAwareMap : public VoxelValueMap {
 public:
  explicit UncertaintyAwareMap(PlannerI& planner);  // NOLINT

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

  // accessor to the server for specialized planners
  voxblox::EsdfServer& getESDFServer();

 protected:
  static ModuleFactoryRegistry::Registration<UncertaintyAwareMap> registration;

  // Map used for reconstruction
  VoxbloxMap voxbloxMap;
  // esdf server that contains the map, subscribe to external ESDF/TSDF updates
  std::unique_ptr<voxblox::EsdfServer> uncertainty_aware_esdf_server;

  // cache constants
  double c_voxel_size_;
  double c_block_size_;
  double c_maximum_weight_;
};

}  // namespace map
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_APP_CURIOSITY_UNCERTAINTY_AWARE_MAP_H
