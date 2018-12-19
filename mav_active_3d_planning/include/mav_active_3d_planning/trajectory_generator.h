#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_

#include "mav_active_3d_planning/trajectory_segment.h"

#include <ros/node_handle.h>
#include <voxblox_ros/esdf_server.h>
#include <vector>

namespace mav_active_3d_planning {

    // Base class for trajectory generation to provide uniform interface with other classes
    class TrajectoryGenerator {
    public:
        TrajectoryGenerator(voxblox::EsdfServer *voxblox_Ptr, bool p_coll_optimistic, double p_coll_radius)
                : voxblox_Ptr_(voxblox_Ptr),
                  p_coll_optimistic_(p_coll_optimistic),
                  p_coll_radius_(p_coll_radius) {}

        virtual ~TrajectoryGenerator() {}

        // Expand a selected trajectory segment and add the new trajectories to result
        virtual bool expandSegment(std::vector<TrajectorySegment> *result, TrajectorySegment start) {
            ROS_ERROR("expandSegments function not implemented.");
            return false;
        }

        // Set params from ROS Nodehandle.
        virtual bool setParamsFromRos(const ros::NodeHandle &nh) {
            ROS_ERROR("setParamsFromRos function not implemented.");
            return false;
        }

    protected:
        // Pointer to the esdf server for collision checking (and others)
        voxblox::EsdfServer *voxblox_Ptr_;

        // Whether unobserved space should be rated as free or occupied
        bool p_coll_optimistic_;

        // Collision radius
        double p_coll_radius_;

        // Returns true if the position is traversable
        bool checkCollision(const Eigen::Vector3d &position) {
            double distance = 0.0;
            if (voxblox_Ptr_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance) &&
                voxblox_Ptr_->getEsdfMapPtr()->isObserved(position)) {
                return (distance > p_coll_radius_);
            }
            return p_coll_optimistic_;
        }
    };

}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
