#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_SEGMENT_H_
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_SEGMENT_H_

#include <mav_msgs/eigen_mav_msgs.h>
#include <vector>

namespace mav_active_3d_planning {

    // Struct to store trajectory tree data
    struct TrajectorySegment {
        TrajectorySegment() {};

        // All trajectory points
        mav_msgs::EigenTrajectoryPointVector trajectory;

        // Associated cost
        double cost;

        // pointer to parent trajectory, nullptr for currently active segment
        TrajectorySegment *parent;

        // Pointers to successive trajectory nodes
        std::vector<TrajectorySegment *> children;

        // compare function for sorting etc
        static bool compare(TrajectorySegment a, TrajectorySegment b) {
            return (a.cost > b.cost);
        }
    };

}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_SEGMENT_H_
