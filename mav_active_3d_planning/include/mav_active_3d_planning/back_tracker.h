#ifndef MAV_ACTIVE_3D_PLANNING_BACK_TRACKER_H_
#define MAV_ACTIVE_3D_PLANNING_BACK_TRACKER_H_

#include "mav_active_3d_planning/trajectory_segment.h"

namespace mav_active_3d_planning {

    // Base class for backtrackers to provide uniform interface with other classes
    // A backtracker's responsibility is to handle the states where no new trajectories were found for execution.
    class BackTracker {
    public:
        // This function lets the backtracker know when a trajectory is executed
        virtual bool segmentIsExecuted(TrajectorySegment segment) {
            return false; // Default does not register movement
        }

        // This function is executed when no new trajectories are found.
        virtual bool trackBack(TrajectorySegment &target) = 0;
    };
}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_BACK_TRACKER_H_
