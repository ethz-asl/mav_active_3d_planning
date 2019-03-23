#ifndef MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_DEFAULT_GENERATOR_UPDATERS_H
#define MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_DEFAULT_GENERATOR_UPDATERS_H

#include "mav_active_3d_planning/trajectory_generator.h"

namespace mav_active_3d_planning {
    namespace generator_updaters {

        // Discard all segments and start from scratch
        class GeneratorResetTree : public GeneratorUpdater {
        public:
            GeneratorResetTree() {}

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

        protected:
            friend ModuleFactory;

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<GeneratorResetTree> registration;
        };

        // Don't perform specific update operations
        class GeneratorUpdateNothing : public GeneratorUpdater {
        public:
            GeneratorUpdateNothing() {}

            bool updateSegments(TrajectorySegment *root) { return true; }

        protected:
            friend ModuleFactory;

            void setupFromParamMap(Module::ParamMap *param_map) {}

            static ModuleFactory::Registration<GeneratorUpdateNothing> registration;
        };

        // Recursively check wether the trajectories are still collision free
        class RecheckCollision : public GeneratorUpdater {
        public:
            RecheckCollision(TrajectoryGenerator *parent);

            // override virtual functions
            bool updateSegments(TrajectorySegment *root);

            static ModuleFactory::Registration<RecheckCollision> registration;

        protected:
            friend ModuleFactory;

            // factory access
            RecheckCollision() {}
            void setupFromParamMap(Module::ParamMap *param_map) {}

            // methods
            bool isCollided(const mav_msgs::EigenTrajectoryPointVector &trajectory);
            void checkSingle(TrajectorySegment *segment);
        };

    } // namespace generator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_DEFAULT_GENERATOR_UPDATERS_H

