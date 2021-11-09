#include "active_3d_planning_panoptic/map/single_tsdf_panoptic_map.h"

#include <cmath>

#include <voxblox/interpolator/interpolator.h>

#include "active_3d_planning_core/data/system_constraints.h"
namespace active_3d_planning {
namespace map {

ModuleFactoryRegistry::Registration<PanopticMap> PanopticMap::registration(
    "SingleTsdfPanopticMap");

PanopticMap::PanopticMap(PlannerI& planner) : VoxelValueMap(planner) {}

void PanopticMap::setupFromParamMap(Module::ParamMap* param_map) {
  std::string uncertainty_str;
  setParam<std::string>(param_map, "value_function", &uncertainty_str,
                        "uncertainty");
  if (uncertainty_str == "uncertainty") {
    value_type = UNCERTAINTY;
  } else if (uncertainty_str == "voxel_entropy") {
    value_type = VOXEL_ENTROPY;
  } else if (uncertainty_str == "voxel_probability") {
    value_type = VOXEL_PROBABILITY;
  } else {
    LOG(WARNING) << "Invalid value function " << uncertainty_str
                 << " going to use default uncertainty. Available Value "
                    "functions: [uncertainty, voxel_entropy, voxel_probability]"
                 << std::endl;
  }
}

const panoptic_mapping::Submap* PanopticMap::getPlanningSubmap() {
  if (!mapper_ ||
      !mapper_->getSubmapCollection().submapIdExists(
          mapper_->getSubmapCollection().getActiveFreeSpaceSubmapID())) {
    ROS_ERROR_THROTTLE(
        1, "Mapper or freespace submap does not exists in planner map");
    return nullptr;
  }
  return &mapper_->getSubmapCollection().getSubmap(
      mapper_->getSubmapCollection().getActiveFreeSpaceSubmapID());
}
bool PanopticMap::isTraversable(const Eigen::Vector3d& position,
                                const Eigen::Quaterniond& orientation) {
    // Danger points:
    if ( (position.x() - 1.5) < 0.6 && (position.x() - 1.5) > -0.6 &&
         (position.y() - 1.3) < 0.6 && (position.y() - 1.3) > -0.6) {
        return false;
    }
    if ( (position.x() - 1.7) < 0.6 && (position.x() - 1.7) > -0.6 &&
         (position.y() - 1.55) < 0.6 && (position.y() - 1.55) > -0.6) {
        return false;
    }
    // Danger points:
    if ( (position.x() - 0.8) < 0.6 && (position.x() - 0.8) > -0.6 &&
            (position.y() - 1.7) < 0.6 && (position.y() - 1.7) > -0.6) {
        return false;
    }
    if ( (position.x() - 1.0) < 0.6 && (position.x() -  (1.0)) > -0.6 &&
         (position.y() - 2) < 0.4 && (position.y() - 2) > -0.4) {
        return false;
    }
    // wtf point
    if ( (position.x() - (-6.3)) < 0.2 && (position.x() -  (-6.3)) > -0.2 &&
         (position.y() - 0.6) < 0.2 && (position.y() - 0.6) > -0.2) {
        return false;
    }
    // wtf point
    if ( (position.x() - (-6.0)) < 0.2 && (position.x() -  (-6.0)) > -0.2 &&
         (position.y() - 0.3) < 0.2 && (position.y() - 0.3) > -0.2) {
        return false;
    }
    // Stairs
    if ( (position.x() - (-6.8)) < 0.6 && (position.x() -  (-6.8)) > -0.6 &&
         (position.y() - (-0.44)) < 0.6 && (position.y() - (-0.44)) > -0.6) {
        return false;
    }

    if  (position.y()  < - 0.17) { // STair check
        return false;
    }
  Eigen::Vector3d robot_feet = position;
    robot_feet.z() -= 0.40;
  return getVoxelDistance(position) >
         planner_.getSystemConstraints().collision_radius && getVoxelDistance(robot_feet) >
                                                             planner_.getSystemConstraints().collision_radius;
}

bool PanopticMap::isObserved(const Eigen::Vector3d& point) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map) return false;

  const auto position_S = map->getT_S_M() * point.cast<float>();
  auto block_ptr = map->getTsdfLayer().getBlockPtrByCoordinates(position_S);
  if (block_ptr) {
    return (block_ptr->getVoxelByCoordinates(position_S).weight >=
            kObservedMinWeight_);
  }
  return false;
}

int PanopticMap::getVoxelObservedCount(const Eigen::Vector3d& point) {
    panoptic_mapping::ClassVoxelType voxel;
    if(!getClassVoxelAt(point, voxel)) {
        return 0;
    }
    return voxel.belongs_count + voxel.foreign_count;
}

    // get occupancy
unsigned char PanopticMap::getVoxelState(const Eigen::Vector3d& point) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map) return false;
  const auto position_S = map->getT_S_M() * point.cast<float>();

  auto block_ptr = map->getTsdfLayer().getBlockPtrByCoordinates(position_S);
  if (!block_ptr) {
    return OccupancyMap::UNKNOWN;
  }

  const auto& voxel = block_ptr->getVoxelByCoordinates(position_S);

  if (voxel.weight <= kObservedMinWeight_) {
    return OccupancyMap::UNKNOWN;
  }

  if (voxel.distance >= map->getTsdfLayer().voxel_size()) {
    return OccupancyMap::FREE;
  }

  return OccupancyMap::OCCUPIED;
}

// get voxel size
double PanopticMap::getVoxelSize() {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map) return 0.1;
  return map->getTsdfLayer().voxel_size();
}

// get the center of a voxel from input point
bool PanopticMap::getVoxelCenter(Eigen::Vector3d* center,
                                 const Eigen::Vector3d& point) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map) return false;

  // Look up the voxel if it is observed.
  const auto position_S = map->getT_S_M() * point.cast<float>();
  auto block = map->getTsdfLayer().getBlockPtrByCoordinates(position_S);
  if (!block) return false;
  auto v_index = block->computeVoxelIndexFromCoordinates(position_S);
  if (!block->isValidVoxelIndex(v_index)) return false;
  *center = (block->computeCoordinatesFromVoxelIndex(v_index)).cast<double>();
  return true;
}

// get the stored TSDF distance
double PanopticMap::getVoxelDistance(const Eigen::Vector3d& point) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map || !mapper_->getEsdfMapPtr()) return 0.0;
  double current_distance = 0.0;

  // Look up the voxel if it is observed.
  const auto position_S = map->getT_S_M() * point.cast<float>();
  mapper_->getEsdfMapPtr()->getDistanceAtPosition(position_S.cast<double>(),
                                                  &current_distance);
  return current_distance;
}

// get the stored weight
double PanopticMap::getVoxelWeight(const Eigen::Vector3d& point) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map || !mapper_->getEsdfMapPtr()) return 0.0;

  // Look up the voxel if it is observed.
  const auto position_S = map->getT_S_M() * point.cast<float>();
  auto block_ptr = map->getTsdfLayer().getBlockPtrByCoordinates(position_S);
  if (!block_ptr) {
    return 0.0;
  }
  const auto& voxel = block_ptr->getVoxelByCoordinates(position_S);
  return voxel.weight;
}

bool PanopticMap::getClassVoxelAt(const Eigen::Vector3d& point,
                                  panoptic_mapping::ClassVoxelType& voxel) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map || !mapper_->getEsdfMapPtr()) return false;

  // Look up the voxel if it is observed.
  const auto position_S = map->getT_S_M() * point.cast<float>();
  auto block_ptr = map->getClassLayer().getBlockPtrByCoordinates(position_S);
  if (!block_ptr) {
    return false;
  }

  voxel = block_ptr->getVoxelByCoordinates(position_S);
  return true;
}
// get the maximum allowed weight (return 0 if using uncapped weights)
double PanopticMap::getMaximumWeight() { return 0.0; }

double PanopticMap::getVoxelValue(const Eigen::Vector3d& point) {
  panoptic_mapping::ClassVoxelType voxel;
  if (!getClassVoxelAt(point, voxel)) {
    LOG(ERROR) << "Tried to access invalid voxel";
    return 0.0;
  }

  switch (value_type) {
    case UNCERTAINTY:
      return panoptic_mapping::classVoxelUncertainty(voxel);
    case VOXEL_ENTROPY:
      return panoptic_mapping::classVoxelEntropy(voxel);
    case VOXEL_PROBABILITY:
      return 1 - panoptic_mapping::classVoxelBelongingProbability(voxel);
    default:
      return 0.0;
  }
}
unsigned char PanopticMap::getValueVoxelState(const Eigen::Vector3d& point) {
  const panoptic_mapping::Submap* map = getPlanningSubmap();
  if (!map || !mapper_->getEsdfMapPtr()) return 0.0;

  const auto position_S = map->getT_S_M() * point.cast<float>();
  auto block_ptr = map->getTsdfLayer().getBlockPtrByCoordinates(position_S);
  auto c_block_ptr = map->getClassLayer().getBlockPtrByCoordinates(position_S);
  if (!block_ptr || !c_block_ptr) {
    return OccupancyMap::UNKNOWN;
  }

  const auto& voxel = block_ptr->getVoxelByCoordinates(position_S);
  if (voxel.weight <= kObservedMinWeight_) {
    return OccupancyMap::UNKNOWN;
  }
  if (voxel.distance >= map->getTsdfLayer().voxel_size()) {
    return OccupancyMap::FREE;
  }
  return OccupancyMap::OCCUPIED;
}
}  // namespace map
}  // namespace active_3d_planning
