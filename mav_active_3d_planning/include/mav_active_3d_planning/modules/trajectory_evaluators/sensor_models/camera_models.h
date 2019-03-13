#ifndef MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_CAMERA_MODELS_H
#define MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_CAMERA_MODELS_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/sensor_model.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>
#include <memory>

namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace sensor_models {
        // Base class for camera models / Utility class that finds visible voxels. Available for all trajectory
        // generators. (improve performance here.)
    class CameraModel : public SensorModel {
        public:
            virtual ~CameraModel() {}

            // Return the voxel centers of all visible voxels for a camera model pointing in x-direction
            virtual bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                          const Eigen::Quaterniond &orientation) = 0;

            // Return the voxel centers of all visible voxels, sampling camera poses from a trajectory segment
            bool
            getVisibleVoxelsFromTrajectory(std::vector <Eigen::Vector3d> *result, const TrajectorySegment &traj_in);

        protected:
            // factory access
            CameraModel() {}

            virtual void setupFromParamMap(Module::ParamMap *param_map);

            virtual bool checkParamsValid(std::string *error_message);

            // parameters
            double p_ray_length_;       // params for camera model
            double p_focal_length_;
            int p_resolution_x_;
            int p_resolution_y_;
            double p_sampling_time_;    // sample camera poses from segment, use 0 for last only

            // constants
            double c_field_of_view_x_;
            double c_field_of_view_y_;
        };

        // Get minimum resolution reuqired for 1 ray per voxel at max distance, then cast a ray through every
        // such voxel to detect visible voxels (timed at 90 +/- 30 ms)
        class SimpleRayCaster : public CameraModel {
        public:
            virtual ~SimpleRayCaster() {}

            // Override virutal functions
            bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                  const Eigen::Quaterniond &orientation);

        protected:
            friend ModuleFactory;

            // factory access
            SimpleRayCaster() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // params
            double p_ray_step_;

            // constants
            int c_res_x_;       // factual resolution that is used for ray casting
            int c_res_y_;
        };

        // Cast Rays until they span 1 full voxel, then duplicate them to omit redundancies
        // (timed at 55 +/- 25 ms)
        class IterativeRayCaster : public CameraModel {
        public:
            virtual ~IterativeRayCaster() {}

            // Override virutal functions
            bool getVisibleVoxels(std::vector <Eigen::Vector3d> *result, const Eigen::Vector3d &position,
                                  const Eigen::Quaterniond &orientation);

        protected:
            friend ModuleFactory;

            // factory access
            IterativeRayCaster() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // params
            double p_ray_step_;

            // constants
            int c_res_x_;       // factual resolution that is used for ray casting
            int c_res_y_;
            int c_n_sections_;     // number of ray duplications
            std::vector<double> c_split_distances_;      // distances where rays are duplicated
            std::vector<int> c_split_widths_;            // number of max distance rays that are covered per split

            // variables
            Eigen::ArrayXXi ray_table_;
        };

    } // namespace sensor_models
}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_SENSOR_MODELS_CAMERA_MODELS_H
