#ifndef ACTIVE_3D_PLANNING_CORE_MAP_VOXEL_VALUE_MAP_H_
#define ACTIVE_3D_PLANNING_CORE_MAP_VOXEL_VALUE_MAP_H_

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/map/occupancy_map.h"
#include "active_3d_planning_core/map/tsdf_map.h"
#include "active_3d_planning_core/map/voxel_value_map.h"

namespace active_3d_planning {
namespace map {
// VoxelValueMap is a TSDFMap that additionally stores a single double value for each voxel.
class VoxelValueMap : public TSDFMap {
 public:
  explicit VoxelValueMap(PlannerI& planner) : TSDFMap(planner) {}  // NOLINT

  virtual ~VoxelValueMap() = default;

  virtual double getVoxelValue(const Eigen::Vector3d& point) = 0;
  virtual unsigned char getValueVoxelState(const Eigen::Vector3d &matrix) = 0;
};

}  // namespace map
}  // namespace active_3d_planning

#endif
