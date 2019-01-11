#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_

#include "mav_active_3d_planning/trajectory_segment.h"
#include "trajectory_segment.h"

#include <ros/node_handle.h>
#include <voxblox_ros/esdf_server.h>

#include <vector>
#include <memory>

namespace mav_active_3d_planning {

    // Base class for trajectory generation to provide uniform interface with other classes
    class TrajectoryGenerator {
    public:
        TrajectoryGenerator(voxblox::EsdfServer *voxblox_Ptr, bool p_coll_optimistic, double p_coll_radius)
                : voxblox_Ptr_(voxblox_Ptr),
                  p_coll_optimistic_(p_coll_optimistic),
                  p_coll_radius_(p_coll_radius) {}

        virtual ~TrajectoryGenerator() {}

        // Expansion policy where to expand (from full tree)
        virtual TrajectorySegment* selectSegment(TrajectorySegment &root) {
            ROS_ERROR("TG: selectSegment function not implemented.");
            return nullptr;
        }

        // Expand a selected trajectory segment
        virtual bool expandSegment(TrajectorySegment &target) {
            ROS_ERROR("TG: expandSegment function not implemented.");
            return false;
        }

        // Set params from ROS Nodehandle.
        virtual bool setParamsFromRos(const ros::NodeHandle &nh) {
            ROS_ERROR("TG: setParamsFromRos function not implemented.");
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
            if (!voxblox_Ptr_->getEsdfMapPtr()->isObserved(position)) {
                return p_coll_optimistic_;
            }
            double distance = 0.0;
            voxblox_Ptr_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance);
            return (distance > p_coll_radius_);
        }
    };

}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATOR_H_
