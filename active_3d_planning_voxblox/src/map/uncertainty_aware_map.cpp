#include "active_3d_planning_voxblox/map/uncertainty_aware_map.h"

#include <voxblox_ros/ros_params.h>

#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {
namespace map {

ModuleFactoryRegistry::Registration<UncertaintyAwareMap>
    UncertaintyAwareMap::registration("VoxbloxMapWithUncertainty");

UncertaintyAwareMap::UncertaintyAwareMap(PlannerI& planner)
    : VoxelValueMap(planner), voxbloxMap(planner) {}

voxblox::EsdfServer& UncertaintyAwareMap::getESDFServer() {
  return voxbloxMap.getESDFServer();
}

void UncertaintyAwareMap::setupFromParamMap(Module::ParamMap* param_map) {
  // Reconstruction + collision map
  voxbloxMap.setupFromParamMap(param_map);

  std::string uncertainty_ns;
  setParam<std::string>(param_map, "uncertainty_namespace", &uncertainty_ns,
                   "/uncertainty/map/");
  // Uncertainty Map
  ros::NodeHandle nh(uncertainty_ns);
  ros::NodeHandle nh_private(uncertainty_ns);

  uncertainty_aware_esdf_server.reset(new voxblox::EsdfServer(nh, nh_private));
  uncertainty_aware_esdf_server->setTraversabilityRadius(
      planner_.getSystemConstraints().collision_radius);

  // cache constants
  c_voxel_size_ = uncertainty_aware_esdf_server->getEsdfMapPtr()->voxel_size();
  c_block_size_ = uncertainty_aware_esdf_server->getEsdfMapPtr()->block_size();
  c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)
                          .max_weight;  // direct access is not exposed
}

bool UncertaintyAwareMap::isTraversable(const Eigen::Vector3d& position,
                                        const Eigen::Quaterniond& orientation) {
  return voxbloxMap.isTraversable(position, orientation);
}

bool UncertaintyAwareMap::isObserved(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  return voxbloxMap.isObserved(point);
}

// get occupancy
unsigned char UncertaintyAwareMap::getVoxelState(const Eigen::Vector3d& point) {
  return voxbloxMap.getVoxelState(point);
}

// get voxel size
double UncertaintyAwareMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool UncertaintyAwareMap::getVoxelCenter(Eigen::Vector3d* center,
                                         const Eigen::Vector3d& point) {
  return voxbloxMap.getVoxelCenter(center, point);
}

// Returns the uncertainty value stored in the voxel
double UncertaintyAwareMap::getVoxelValue(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      uncertainty_aware_esdf_server->getTsdfMapPtr()
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);

  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    return ((double) tsdf_voxel->color.b) / 255.0;
  }

  return -1;
}

// get the stored TSDF distance
double UncertaintyAwareMap::getVoxelDistance(const Eigen::Vector3d& point) {
  return voxbloxMap.getVoxelDistance(point);
}

// get the stored weight

double UncertaintyAwareMap::getVoxelWeight(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
          uncertainty_aware_esdf_server->getTsdfMapPtr()
                  ->getTsdfLayerPtr()
                  ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
            block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->weight;
    }
  }
  return 0.0;
}

// get the maximum allowed weight (return 0 if using uncapped weights)
double UncertaintyAwareMap::getMaximumWeight() {
  return voxbloxMap.getMaximumWeight();
}



// get occupancy
unsigned char UncertaintyAwareMap::getValueVoxelState(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (uncertainty_aware_esdf_server->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < 2.5 * c_voxel_size_) {
      return VoxbloxMap::OCCUPIED;
    } else {
      return VoxbloxMap::FREE;
    }
  } else {
    return VoxbloxMap::UNKNOWN;
  }
}
}  // namespace map
}  // namespace active_3d_planning
