#ifndef MAV_ACTIVE_3D_PLANNING_GLOBAL_NORMALIZED_GAIN_VALUE_COMPUTERS_H
#define MAV_ACTIVE_3D_PLANNING_GLOBAL_NORMALIZED_GAIN_VALUE_COMPUTERS_H

#include "active_3d_planning_core/module/module_factory_registry.h"
#include "active_3d_planning_core/module/trajectory_evaluator.h"

namespace active_3d_planning {
    namespace value_computer {

        // Use the best subsequent value for each segment in the subtree
        class GlobalNormalizedGain : public ValueComputer {
        public:
            GlobalNormalizedGain(PlannerI &planner);

            // override virtual functions
            bool computeValue(TrajectorySegment *traj_in) override;

            void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<GlobalNormalizedGain> registration;

            // methods
            double findBest(TrajectorySegment *current, double gain, double cost);
        };

    } // namespace value_computer
} // namespace active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_GLOBAL_NORMALIZED_GAIN_VALUE_COMPUTERS_H