#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_LIDAR_MODEL_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_LIDAR_MODEL_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/sensor_model.h"

namespace mav_active_3d_planning {
    namespace sensor_models {

        // Base class for camera models / Utility class that finds visible voxels. Available for all trajectory
        // generators. (improve performance here.)
        class LidarModel : public SensorModel {
        public:
            virtual ~LidarModel() {}

            // Return the voxel centers of all visible voxels for a camera model pointing in x-direction
            virtual bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                          const Eigen::Quaterniond &orientation) = 0;

            // Return the voxel centers of all visible voxels, sampling camera poses from a trajectory segment
            bool
            getVisibleVoxelsFromTrajectory(std::vector <Eigen::Vector3d> *result, const TrajectorySegment &traj_in);

            // Display camera view bounds
            void visualizeSensorView(visualization_msgs::MarkerArray *msg, const TrajectorySegment &traj_in);

        protected:
            // factory access
            LidarModel() {}

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            virtual bool checkParamsValid(std::string *error_message);

            // parameters
            double p_ray_length_;       // params for camera model
            double p_fov_x_;            // Total fields of view [deg], expected symmetric w.r.t. sensor facing direction
            double p_fov_y_;
            int p_resolution_x_;
            int p_resolution_y_;
            double p_sampling_time_;    // sample camera poses from segment, use 0 for last only

            // For performance timing of different implementations
            bool p_test_;
            std::vector<double> time_count_;

            // methods
            // return the indices of the trajectory points of traj_in for which a view is needed
            void sampleViewpoints(std::vector<int> *result, const TrajectorySegment &traj_in);
            void visualizeSingleView(visualization_msgs::MarkerArray *msg, const Eigen::Vector3d &position,
                                     const Eigen::Quaterniond &orientation);
            // get the direction vector for camera pointing in x_direction at pixel with relative x, y position [0, 1]
            void getDirectionVector(Eigen::Vector3d *result, double relative_x, double relative_y);
        };

    } // namespace sensor_models
}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_LIDAR_MODEL_H
