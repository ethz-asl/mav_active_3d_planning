#ifndef MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_SIMPLE_RRT_STAR_UPDATER_H
#define MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_SIMPLE_RRT_STAR_UPDATER_H

#include "mav_active_3d_planning/modules/trajectory_generators/rrt_star.h"

namespace mav_active_3d_planning {
    namespace generator_updaters {

        // Recursively check wether the trajectories are still collision free, if not try to rewire them
        class SimpleRRTStarCollisionUpdater : public GeneratorUpdater {
        public:
            // override virtual functions
            bool updateSegments(TrajectorySegment *root) override;

        protected:
            friend ModuleFactory;

            // factory access
            SimpleRRTStarCollisionUpdater() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration <SimpleRRTStarCollisionUpdater> registration;

            // reference to Generator for rewiring
            mav_active_3d_planning::trajectory_generators::RRTStar *generator_;

            // members
            std::unique_ptr <GeneratorUpdater> following_updater_;

            // methods
            bool isCollided(const mav_msgs::EigenTrajectoryPointVector &trajectory);

            void checkSingle(TrajectorySegment *segment, std::vector<TrajectorySegment *> *to_rewire, std::vector<TrajectorySegment *> *safe_segments);
        };

    } // namespace generator_updaters
} // namepsace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_GENERATOR_UPDATERS_SIMPLE_RRT_STAR_UPDATER_H

