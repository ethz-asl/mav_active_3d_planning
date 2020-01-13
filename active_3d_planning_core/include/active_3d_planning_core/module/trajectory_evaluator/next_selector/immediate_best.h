#ifndef ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_IMMEDIATE_BEST_H
#define ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_IMMEDIATE_BEST_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace next_selector {

        // Select the child node which has the highest value
        class ImmediateBest : public NextSelector {
        public:
            ImmediateBest(PlannerI &planner);

            // override virtual functions
            int selectNextBest(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<ImmediateBest> registration;
        };

    } // namespace next_selector
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_IMMEDIATE_BEST_H
