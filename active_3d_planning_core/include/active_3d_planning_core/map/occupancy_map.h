#ifndef ACTIVE_3D_PLANNING_CORE_MAP_OCCUPANCY_MAP_H
#define ACTIVE_3D_PLANNING_CORE_MAP_OCCUPANCY_MAP_H

#include "active_3d_planning_core/map/map.h"
#include "active_3d_planning_core/data/trajectory.h"

namespace active_3d_planning {
    namespace map {

        // base for occupancy grid maps
        class OccupancyMap : public Map {
        public:
            OccupancyMap(PlannerI &planner) : Map(planner) {}

            virtual ~OccupancyMap() = default;

            // states
            const static unsigned char OCCUPIED = 0;
            const static unsigned char FREE = 1;
            const static unsigned char UNKNOWN = 2;

            // get occupancy
            virtual unsigned char getVoxelState(const Eigen::Vector3d &point) = 0;

            // get voxel size
            virtual double getVoxelSize() = 0;

            // get the center of a voxel from input point
            virtual bool getVoxelCenter(Eigen::Vector3d *center, const Eigen::Vector3d &point) = 0;
        };

    } // namespace map
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_CORE_MAP_OCCUPANCY_MAP_H
