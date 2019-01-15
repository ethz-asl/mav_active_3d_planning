#ifndef MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
#define MAV_ACTIVE_3D_PLANNING_DEFAULTS_H

#include "mav_active_3d_planning/trajectory_segment.h"

namespace mav_active_3d_planning {
    namespace defaults {
        /* Contains default methods and utility functions that can be used by various trajectory generators and/or
         * evaluators. */

        /* Trajectory Generators: selectSegment functions */
        // Select a random leaf of the trajectory tree
        TrajectorySegment *selectRandomLeaf(TrajectorySegment &root);

        // Select a random leaf of the trajectory tree, every leaf is weighted with its value
        TrajectorySegment* selectRandomLeafWeighted(TrajectorySegment &root, double uniform_weight);

    } // namespace defaults
} // namepsace mav_active_3d_planning

#endif //MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
