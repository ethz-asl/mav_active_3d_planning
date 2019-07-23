#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_MAV_TRAJECTORY_GENERATION_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_MAV_TRAJECTORY_GENERATION_H

#include "mav_active_3d_planning/trajectory_generator.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Create random trajectories using the mav_trajectory_generation tools and check for MAV constraints (such as max
        // yaw rate, thrusts, ...)
        class DiffDriveTrajectoryGeneration : public TrajectoryGenerator {
        public:
            // override virtual functions
            bool expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments);

        protected:
            friend ModuleFactory;

            // factory access
            DiffDriveTrajectoryGeneration() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            bool checkParamsValid(std::string *error_message);

            static ModuleFactory::Registration <DiffDriveTrajectoryGeneration> registration;

            // methods
            void initializeConstraints();

            // parameters
            double p_distance_max_;
            double p_distance_min_;
            double p_sampling_rate_;
            int p_n_segments_;
            int p_max_tries_;

            // optimization settings
            mav_trajectory_generation::FeasibilityAnalytic feasibility_check_;
            mav_trajectory_generation::NonlinearOptimizationParameters parameters_;

            // methods
            void sampleGoalPose(double *yaw, Eigen::Vector3d *goal_pos, const Eigen::Vector3d &start_pos);

            void optimizeVertices(mav_trajectory_generation::Vertex::Vector *vertices,
                                  mav_trajectory_generation::Segment::Vector *segments,
                                  mav_trajectory_generation::Trajectory *trajectory);

            bool checkInputFeasible(const mav_trajectory_generation::Segment::Vector &segments);

            bool checkTrajectoryCollision(const mav_msgs::EigenTrajectoryPoint::Vector &states);
        };

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_MAV_TRAJECTORY_GENERATION_H
