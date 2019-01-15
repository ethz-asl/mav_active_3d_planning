#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_segment.h"

#include <ros/node_handle.h>
#include <voxblox_ros/esdf_server.h>

namespace mav_active_3d_planning {

    // Base class for trajectory evaluation to provide uniform interface with other classes
    class TrajectoryEvaluator {
    public:
        TrajectoryEvaluator(voxblox::EsdfServer *voxblox_Ptr)
                : voxblox_Ptr_(voxblox_Ptr) {}

        virtual ~TrajectoryEvaluator() {}

        // compute the gain for a TrajectorySegment
        virtual bool computeGain(TrajectorySegment &traj_in) {
            ROS_ERROR("TE: computeGain function not implemented.");
            return false;
        }

        // compute the cost for a TrajectorySegment
        virtual bool computeCost(TrajectorySegment &traj_in) {
            ROS_ERROR("TE: computeCost function not implemented.");
            return false;
        }

        // compute the Value for a segment with known cost and gain
        virtual bool computeValue(TrajectorySegment &traj_in) {
            ROS_ERROR("TE: computeDifference function not implemented.");
            return false;
        }

        // return the index of the most promising child segment
        virtual int selectNextBest(TrajectorySegment &traj_in) {
            ROS_ERROR("TE: selectNextBest function not implemented.");
            return -1;
        }

        // Set params from ROS Nodehandle.
        virtual bool setParamsFromRos(const ros::NodeHandle &nh) {
            ROS_ERROR("TE: setParamsFromRos function not implemented.");
            return false;
        }

    protected:
        // Pointer to the esdf server for collision checking (and others)
        voxblox::EsdfServer *voxblox_Ptr_;
    };
}  // namespace mav_active_3d_planning

#endif //MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATOR_H
