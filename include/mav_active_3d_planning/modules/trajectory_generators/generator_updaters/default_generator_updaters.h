#ifndef MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_DEFAULT_GENERATOR_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_DEFAULT_GENERATOR_UPDATERS_H

#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/planner_node.h"

namespace mav_active_3d_planning {
    namespace generator_updaters {

        // Discard all segments and start from scratch
        class GeneratorResetTree : public GeneratorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            GeneratorResetTree() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<GeneratorResetTree> registration;
        };

        // Don't perform specific update operations
        class GeneratorUpdateNothing : public GeneratorUpdater {
        public:
            bool updateSegments(TrajectorySegment *root) { return true; }

        protected:
            friend ModuleFactory;

            GeneratorUpdateNothing() {}

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<GeneratorUpdateNothing> registration;
        };

        // Recursively check wether the trajectories are still collision free
        class RecheckCollision : public GeneratorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            // factory access
            RecheckCollision() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<RecheckCollision> registration;

            // methods
            bool isCollided(const mav_msgs::EigenTrajectoryPointVector &trajectory);
            void checkSingle(TrajectorySegment *segment);

            // pointer to the planner node to request collision checks
            PlannerNode* planner_node_;
        };

    } // namespace generator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_DEFAULT_GENERATOR_UPDATERS_H

