#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>

namespace mav_active_3d_planning {

    // Abstract wrapper for default/modular implementations of the computeCost method
    class CostComputer {
    public:
        virtual bool computeCost(TrajectorySegment &traj_in) = 0;
    };

    // Abstract wrapper for default/modular implementations of the computeValue method
    class ValueComputer {
    public:
        virtual bool computeValue(TrajectorySegment &traj_in) = 0;
    };

    // Abstract wrapper for default/modular implementations of the selectNextBest method
    class NextSelector {
    public:
        virtual int selectNextBest(TrajectorySegment &traj_in) = 0;
    };

    // Base class for trajectory evaluators to provide uniform interface with other classes
    class TrajectoryEvaluator {
    public:
        TrajectoryEvaluator(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

        virtual ~TrajectoryEvaluator() {}

        // compute the gain of a TrajectorySegment
        virtual bool computeGain(TrajectorySegment &traj_in);

        // compute the cost of a TrajectorySegment
        virtual bool computeCost(TrajectorySegment &traj_in);

        // compute the Value of a segment with known cost and gain
        virtual bool computeValue(TrajectorySegment &traj_in);

        // return the index of the most promising child segment
        virtual int selectNextBest(TrajectorySegment &traj_in);

    protected:
        // Voxblox map
        voxblox::EsdfServer *voxblox_ptr_;

        // bounding volume of interesting target
        defaults::BoundingVolume bounding_volume_;

        // params
        std::string p_namespace_;

        // default modules
        CostComputer* cost_computer_;
        ValueComputer* value_computer_;
        NextSelector* next_selector_;
    };

    // Utility class that finds visible voxels. Available for all trajectory generators, improve performance here.
    class RayCaster {
    public:
        RayCaster(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

        virtual ~RayCaster() {}

        // Return the voxel centers of all visible voxels for a simple camera model pointing in x-direction
        std::vector <Eigen::Vector3d> getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation);

    protected:
        // voxblox map
        voxblox::EsdfServer *voxblox_ptr_;

        // parameters
        double p_ray_length_;
        double p_focal_length_;
        double p_ray_step_;
        int p_resolution_x_;
        int p_resolution_y_;

        // constants
        voxblox::FloatingPoint c_voxel_size_;
        voxblox::FloatingPoint c_block_size_;
        double c_field_of_view_x_;
        double c_field_of_view_y_;
    };

}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
