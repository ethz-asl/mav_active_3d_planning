#ifndef ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_LENGTH_H
#define ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_LENGTH_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace cost_computer {

        // Travelled distance of a segment
        class SegmentLength : public CostComputer {
        public:
            explicit SegmentLength(PlannerI &planner);

            void setupFromParamMap(Module::ParamMap *param_map) override;

            // override virtual functions
            bool computeCost(TrajectorySegment *traj_in) override;

        protected:
            static ModuleFactoryRegistry::Registration<SegmentLength> registration;

            // params
            bool p_accumulate_; // True: Use total length
        };

    } // namespace cost_computer
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_COST_COMPUTERS_SEGMENT_LENGTH_H
