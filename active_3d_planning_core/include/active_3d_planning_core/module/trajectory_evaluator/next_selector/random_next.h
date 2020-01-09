#ifndef ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_RANDOM_NEXT_H
#define ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_RANDOM_NEXT_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace next_selector {

// Select the child node which has the highest value
        class RandomNext : public NextSelector {
        public:
            RandomNext(PlannerI &planner);

            // override virtual functions
            int selectNextBest(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<RandomNext> registration;
        };

    } // namespace next_selector
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_RANDOM_NEXT_H
