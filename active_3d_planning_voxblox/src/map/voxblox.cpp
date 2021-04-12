#include "active_3d_planning_voxblox/map/voxblox.h"

#include <voxblox_ros/ros_params.h>

#include "active_3d_planning_core/data/system_constraints.h"

namespace active_3d_planning {
namespace map {

ModuleFactoryRegistry::Registration<VoxbloxMap> VoxbloxMap::registration(
    "VoxbloxMap");

VoxbloxMap::VoxbloxMap(PlannerI& planner) : TSDFMap(planner) {}

voxblox::EsdfServer& VoxbloxMap::getESDFServer() { return *esdf_server_; }

void VoxbloxMap::setupFromParamMap(Module::ParamMap* param_map) {
  // create an esdf server
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));
  esdf_server_->setTraversabilityRadius(
      planner_.getSystemConstraints().collision_radius);

  // cache constants
  c_voxel_size_ = esdf_server_->getEsdfMapPtr()->voxel_size();
  c_block_size_ = esdf_server_->getEsdfMapPtr()->block_size();
  c_maximum_weight_ = voxblox::getTsdfIntegratorConfigFromRosParam(nh_private)
                          .max_weight;  // direct access is not exposed
}

bool VoxbloxMap::isTraversable(const Eigen::Vector3d& position,
                               const Eigen::Quaterniond& orientation) {
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(position,
                                                           &distance)) {
    // This means the voxel is observed
    return (distance > planner_.getSystemConstraints().collision_radius);
  }
  return false;
}

bool VoxbloxMap::isObserved(const Eigen::Vector3d& point) {
  return esdf_server_->getEsdfMapPtr()->isObserved(point);
}

// get occupancy
unsigned char VoxbloxMap::getVoxelState(const Eigen::Vector3d& point) {
  double distance = 0.0;
  if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance)) {
    // This means the voxel is observed
    if (distance < c_voxel_size_) {
      return VoxbloxMap::OCCUPIED;
    } else {
      return VoxbloxMap::FREE;
    }
  } else {
    return VoxbloxMap::UNKNOWN;
  }
}

// get voxel size
double VoxbloxMap::getVoxelSize() { return c_voxel_size_; }

// get the center of a voxel from input point
bool VoxbloxMap::getVoxelCenter(Eigen::Vector3d* center,
                                const Eigen::Vector3d& point) {
  voxblox::BlockIndex block_id = esdf_server_->getEsdfMapPtr()
                                     ->getEsdfLayerPtr()
                                     ->computeBlockIndexFromCoordinates(
                                         point.cast<voxblox::FloatingPoint>());
  *center = voxblox::getOriginPointFromGridIndex(block_id, c_block_size_)
                .cast<double>();
  voxblox::VoxelIndex voxel_id =
      voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
          (point - *center).cast<voxblox::FloatingPoint>(),
          1.0 / c_voxel_size_);
  *center += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_)
                 .cast<double>();
  return true;
}

// get the stored TSDF distance
double VoxbloxMap::getVoxelDistance(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      esdf_server_->getTsdfMapPtr()
          ->getTsdfLayerPtr()
          ->getBlockPtrByCoordinates(voxblox_point);
  if (block) {
    voxblox::TsdfVoxel* tsdf_voxel =
        block->getVoxelPtrByCoordinates(voxblox_point);
    if (tsdf_voxel) {
      return tsdf_voxel->distance;
    }
  }
  return 0.0;
}

// get the stored weight
double VoxbloxMap::getVoxelWeight(const Eigen::Vector3d& point) {
  voxblox::Point voxblox_point(point.x(), point.y(), point.z());
  voxblox::Block<voxblox::TsdfVoxel>::Ptr block =
      esdf_server_->getTsdfMapPtr()
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
double VoxbloxMap::getMaximumWeight() { return c_maximum_weight_; }

}  // namespace map
}  // namespace active_3d_planning
