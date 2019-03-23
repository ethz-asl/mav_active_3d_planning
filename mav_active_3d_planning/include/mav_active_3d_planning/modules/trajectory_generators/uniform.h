#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_UNIFORM_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_UNIFORM_H

#include "mav_active_3d_planning/trajectory_generator.h"

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Sample the input space and propose a uniform grid of new trajectories
        class Uniform : public TrajectoryGenerator {
        public:
            // override virtual functions
            bool expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment*> *new_segments);

        protected:
            friend ModuleFactory;

            // factory access
            Uniform() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<Uniform> registration;

            // parameters
            double p_distance_;         // m
            double p_velocity_;         // m/s
            double p_yaw_angle_;        // rad
            double p_sampling_rate_;    // Hz
            int p_n_segments_;
            double p_ascent_angle_;     // Rad, Set 0 for planar

            // constants
            double c_yaw_rate_;
        };

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_UNIFORM_H
