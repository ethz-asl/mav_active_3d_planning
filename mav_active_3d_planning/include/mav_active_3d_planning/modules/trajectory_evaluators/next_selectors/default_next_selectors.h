#ifndef MAV_ACTIVE_3D_PLANNING_NEXT_SELECTORS_DEFAULT_NEXT_SELECTORS_H
#define MAV_ACTIVE_3D_PLANNING_NEXT_SELECTORS_DEFAULT_NEXT_SELECTORS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"

namespace mav_active_3d_planning {
    namespace next_selectors {

        // Select the child node which has the highest value
        class ImmediateBest : public NextSelector {
        public:
            // override virtual functions
            int selectNextBest(const TrajectorySegment &traj_in);

        protected:
            friend ModuleFactory;

            ImmediateBest() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<ImmediateBest> registration;
        };

        // Select the child node which contains the highest value segment in its subtree
        class SubsequentBest : public NextSelector {
        public:
            // override virtual functions
            int selectNextBest(const TrajectorySegment &traj_in);

        protected:
            friend ModuleFactory;

            SubsequentBest() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<SubsequentBest> registration;

            // methods
            double evaluateSingle(TrajectorySegment *traj_in);
        };

    } // namespace next_selectors
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_NEXT_SELECTORS_DEFAULT_NEXT_SELECTORS_H

