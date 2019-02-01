#ifndef MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
#define MAV_ACTIVE_3D_PLANNING_DEFAULTS_H

#include "mav_active_3d_planning/trajectory_segment.h"

#include <random>
#include <vector>


namespace mav_active_3d_planning {
    namespace defaults {
        /* Contains default methods and utility functions that can be used by various trajectory generators and/or
         * evaluators. */

        /* Trajectory Generators: selectSegment functions */
        // Select uniform random segment from candidates
        TrajectorySegment *selectRandomUniform(std::vector<TrajectorySegment *> &candidates);

        // Select random segment weighted with the segment value
        TrajectorySegment *selectRandomWeighted(std::vector<TrajectorySegment *> &candidates);

        // Select a random leaf of the trajectory tree
        TrajectorySegment *selectRandomLeafUniform(TrajectorySegment &root);

        // Select a random leaf of the trajectory tree, every leaf is weighted with its value
        TrajectorySegment *selectRandomLeafWeighted(TrajectorySegment &root);

        // Select a random segment of the trajectory tree
        TrajectorySegment *selectRandomSegmentUniform(TrajectorySegment &root);

        // Select a random segment of the trajectory tree weighted by value
        TrajectorySegment *selectRandomSegmentWeighted(TrajectorySegment &root);

    } // namespace defaults
} // namepsace mav_active_3d_planning

#endif //MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
