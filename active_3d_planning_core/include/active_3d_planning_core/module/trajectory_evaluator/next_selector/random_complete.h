#ifndef ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_RANDOM_COMPLETE_H
#define ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_RANDOM_COMPLETE_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace next_selector {

        // Select the child node which contains the highest value segment in its subtree
        class RandomComplete : public NextSelector {
        public:
            explicit RandomComplete(PlannerI &planner);

            // override virtual functions
            int selectNextBest(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<RandomComplete> registration;

            // methods
            std::vector<TrajectorySegment *> getFromSingle(TrajectorySegment *traj_in);
        };

    } // namespace next_selector
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_NEXT_SELECTOR_RANDOM_COMPLETE_H
