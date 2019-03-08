#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RANDOM_LINEAR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RANDOM_LINEAR_H

#include "mav_active_3d_planning/trajectory_generator.h"

namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace trajectory_generators {

        // Point sampling in space, joined by linear segments
        class RandomLinear : public TrajectoryGenerator {
        public:
            // override virtual functions
            bool expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment*> *new_segments);

        protected:
            friend ModuleFactory;

            // factory access
            RandomLinear() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);

            // params
            double p_min_distance_;     // m
            double p_max_distance_;     // m
            double p_v_max_;            // m/s
            double p_a_max_;            // m/s2
            double p_sampling_rate_;    // Hz
            int p_n_segments_;
            int p_max_tries_;
            bool p_planar_;
            bool p_sample_yaw_;      // false: face direction of travel

            // methods
            void sampleTarget(Eigen::Vector3d *direction, double* yaw, double* decelleration_distance);
            bool buildTrajectory(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &direction,
                                 double yaw, double decelleration_distance, TrajectorySegment* new_segment);
        };

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RANDOM_LINEAR_H
