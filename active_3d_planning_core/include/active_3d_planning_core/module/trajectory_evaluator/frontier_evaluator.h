#ifndef ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_FRONTIER_EVALUATOR_H
#define ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_FRONTIER_EVALUATOR_H

#include "active_3d_planning_core/module/trajectory_evaluator/simulated_sensor_evaluator.h"
#include "active_3d_planning_core/map/occupancy_map.h"

namespace active_3d_planning {
    namespace trajectory_evaluator {

// Frontier counts all unknown voxels which are next to known occupied voxels
        class FrontierEvaluator : public SimulatedSensorEvaluator {
        public:
            FrontierEvaluator(PlannerI &planner);

            // Override virtual methods
            virtual void
            visualizeTrajectoryValue(VisualizationMarkers *markers,
                                     const TrajectorySegment &trajectory) override;

            virtual void setupFromParamMap(Module::ParamMap *param_map) override;

        protected:
            static ModuleFactoryRegistry::Registration<FrontierEvaluator> registration;

            // Override virtual methods
            virtual bool
            computeGainFromVisibleVoxels(TrajectorySegment *traj_in) override;

            // map
            map::OccupancyMap *map_;

            // params
            bool p_accurate_frontiers_; // True: explicitely compute all frontier voxels
            // (may degrade performance),
            // false: estimate frontier voxels by checking only some neighbors (detection
            // depends on previous views)
            bool p_surface_frontiers_; // true: count next to occupied, false: count next
            // to observed
            double p_checking_distance_;    // distance in voxelsizes where we check for known voxels

            // constants
            double c_voxel_size_;
            Eigen::Vector3d c_neighbor_voxels_[26];

            // methods
            bool isFrontierVoxel(const Eigen::Vector3d &voxel);
        };

    } // namespace trajectory_evaluator
} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_TRAJECTORY_EVALUATOR_FRONTIER_EVALUATOR_H
