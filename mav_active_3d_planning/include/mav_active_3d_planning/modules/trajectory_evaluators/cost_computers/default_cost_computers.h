#ifndef MAV_ACTIVE_3D_PLANNING_COST_COMPUTERS_DEFAULT_COST_COMPUTERS_H
#define MAV_ACTIVE_3D_PLANNING_COST_COMPUTERS_DEFAULT_COST_COMPUTERS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"

#include <Eigen/Geometry>

#include <vector>

namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace cost_computers {

        // Execution time of a single segment
        class SegmentTime : public CostComputer {
        public:
            // override virtual functions
            bool computeCost(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            SegmentTime() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}
        };

        // Travelled distance of a segment
        class SegmentLength : public CostComputer {
        public:
            // override virtual functions
            bool computeCost(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            SegmentLength() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}
        };

    } // namespace cost_computers
} // namepsace mav_active_3d_planning

#endif // MAV_ACTIVE_3D_PLANNING_COST_COMPUTERS_DEFAULT_COST_COMPUTERS_H

