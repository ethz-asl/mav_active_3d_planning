#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_FRONTIER_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_FRONTIER_EVALUATOR_H

#include "mav_active_3d_planning/modules/trajectory_evaluators/simulated_sensor_evaluator.h"

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Frontier counts all unknown voxels which are next to known occupied voxels
        class FrontierEvaluator : public SimulatedSensorEvaluator {
        public:
            // Override virtual methods
            virtual void visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory);

        protected:
            friend ModuleFactory;

            // factory access
            FrontierEvaluator() {}

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            static ModuleFactory::Registration<FrontierEvaluator> registration;

            // Override virtual methods
            virtual bool computeGainFromVisibleVoxels(TrajectorySegment *traj_in);

            // params
            bool p_accurate_frontiers_;     // True: explicitely compute all frontier voxels (may degrade performance),
            // false: estimate frontier voxels by checking only some neighbors (detection depends on previous views)
            bool p_surface_frontiers_;      // true: count next to occupied, false: count next to observed
            double p_checking_distance_;    // distance in voxelsizes where we check for known voxels

            // constants
            double c_voxel_size_;
            Eigen::Vector3d c_neighbor_voxels_[26];

            // methods
            bool isFrontierVoxel(const Eigen::Vector3d &voxel);
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_FRONTIER_EVALUATOR_H
