#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_SEGMENT_H_
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_SEGMENT_H_

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <memory>

namespace mav_active_3d_planning {

    // Base struct that contains trajectory evaluator information associated with the segment.
    struct TrajectoryInfo {
        virtual ~TrajectoryInfo() {}
    };

    // Struct to store trajectory tree data
    struct TrajectorySegment {
        TrajectorySegment();

        // All trajectory points
        mav_msgs::EigenTrajectoryPointVector trajectory;

        // Associated costs
        double gain;
        double cost;
        double value;

        // trajectory generator flag
        bool tg_visited;

        // pointer to parent trajectory, nullptr for currently active segment (root)
        TrajectorySegment *parent;

        // Pointers to successive trajectory nodes, all nodes are owned by the parent
        std::vector <std::unique_ptr<TrajectorySegment>> children;

        // Information to be carried with this segment, e.g. virtual voxels
        std::unique_ptr <TrajectoryInfo> info;

        // compare function for sorting etc
        static bool compare(TrajectorySegment a, TrajectorySegment b);

        static bool comparePtr(TrajectorySegment *a, TrajectorySegment *b);

        // Safely create a child node and return a pointer to it
        TrajectorySegment *spawnChild();

        // The following utility functions assume a tree structure (no loops)
        // Add pointers to all immediate children to the result vector
        void getChildren(std::vector<TrajectorySegment *> *result);

        // Recursively add pointers to all leaf nodes (have no children) to the result vector
        void getLeaves(std::vector<TrajectorySegment *> *result);

        // Recursively add pointers to all subsequent nodes to the result vector
        void getTree(std::vector<TrajectorySegment *> *result);

        // Create a shallow copy of the segment (Includes everything except the unique pointers children and info, which
        // will not be setup!)
        TrajectorySegment shallowCopy();
    };

}  // namespace mav_active_3d_planning
#endif  // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_SEGMENT_H_
