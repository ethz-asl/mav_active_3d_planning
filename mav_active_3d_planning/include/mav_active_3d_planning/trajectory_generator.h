#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>

namespace mav_active_3d_planning {

    // Abstract wrapper for default/modular implementations of the selectSegment method
    class SegmentSelector {
    public:
        virtual TrajectorySegment* selectSegment(TrajectorySegment &root) = 0;
    };

    // Base class for trajectory generation to provide uniform interface with other classes
    class TrajectoryGenerator {
    public:
        TrajectoryGenerator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

        virtual ~TrajectoryGenerator() {}

        // Expansion policy where to expand (from full tree)
        virtual TrajectorySegment* selectSegment(TrajectorySegment &root);

        // Expand a selected trajectory segment
        virtual bool expandSegment(TrajectorySegment &target) = 0;

    protected:
        // Pointer to the esdf server for collision checking (and others)
        voxblox::EsdfServer *voxblox_ptr_;

        // bounding box
        defaults::BoundingVolume bounding_volume_;

        // default modules
        SegmentSelector* segment_selector_;

        // Parameters
        bool p_collision_optimistic_;
        double p_collision_radius_;
        std::string p_namespace_;

        // Utility function for collision checking. Returns true if the position is reachable.
        bool checkTraversable(const Eigen::Vector3d &position);
    };
}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
