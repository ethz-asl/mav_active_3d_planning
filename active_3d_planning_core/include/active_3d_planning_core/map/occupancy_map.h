#ifndef ACTIVE_3D_PLANNING_CORE_MAP_OCCUPANCY_MAP_H_
#define ACTIVE_3D_PLANNING_CORE_MAP_OCCUPANCY_MAP_H_

#include "active_3d_planning_core/data/trajectory.h"
#include "active_3d_planning_core/map/map.h"

namespace active_3d_planning {
namespace map {

// base for occupancy grid maps
class OccupancyMap : public Map {
 public:
  explicit OccupancyMap(PlannerI& planner) : Map(planner) {}  // NOLINT
  virtual ~OccupancyMap() = default;

  // states
  const static unsigned char OCCUPIED = 0;  // NOLINT
  const static unsigned char FREE = 1;      // NOLINT
  const static unsigned char UNKNOWN = 2;   // NOLINT

  // get occupancy
  virtual unsigned char getVoxelState(const Eigen::Vector3d& point) = 0;

  // get voxel size
  virtual double getVoxelSize() = 0;

  // get the center of a voxel from input point
  virtual bool getVoxelCenter(Eigen::Vector3d* center,
                              const Eigen::Vector3d& point) = 0;
};

}  // namespace map
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MAP_OCCUPANCY_MAP_H_
