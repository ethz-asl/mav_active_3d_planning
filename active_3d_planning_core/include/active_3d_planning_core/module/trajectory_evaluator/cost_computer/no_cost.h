#ifndef ACTIVE_3D_PLANNING_COST_COMPUTERS_NO_COST_H
#define ACTIVE_3D_PLANNING_COST_COMPUTERS_NO_COST_H

#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace cost_computer {

        class NoCost : public CostComputer {
        public:
            explicit NoCost(PlannerI &planner);

            // override virtual functions
            bool computeCost(TrajectorySegment *traj_in) override;

        protected:
            void setupFromParamMap(Module::ParamMap *param_map) override;

            static ModuleFactoryRegistry::Registration<NoCost> registration;
        };

    } // namespace cost_computer
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_COST_COMPUTERS_NO_COST_H
