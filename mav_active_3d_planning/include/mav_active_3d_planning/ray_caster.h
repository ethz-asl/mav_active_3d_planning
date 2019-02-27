#ifndef MAV_ACTIVE_3D_PLANNING_RAY_CASTER_H
#define MAV_ACTIVE_3D_PLANNING_RAY_CASTER_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <voxblox_ros/esdf_server.h>
#include <Eigen/Core>

#include <string>

namespace mav_active_3d_planning {
//    namespace ray_casters {

        // Utility class that finds visible voxels. Available for all trajectory generators. (improve performance here.)
        class RayCaster {
        public:
            RayCaster(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            virtual ~RayCaster() {}

            // Return the voxel centers of all visible voxels for a simple camera model pointing in x-direction
            std::vector <Eigen::Vector3d>
            getVisibleVoxels(Eigen::Vector3d position, Eigen::Quaterniond orientation);

            // Return the voxel centers of all visible voxels, sampling camera poses from a trajectory segment
            std::vector <Eigen::Vector3d> getVisibleVoxelsFromTrajectory(TrajectorySegment *traj_in);

        protected:
            // voxblox map
            voxblox::EsdfServer *voxblox_ptr_;

            // parameters
            double p_ray_length_;       // params for simple camera model
            double p_focal_length_;
            double p_ray_step_;
            int p_resolution_x_;
            int p_resolution_y_;
            double p_sampling_time_;    // sample camera poses from segment, use 0 for alst only

            // constants
            voxblox::FloatingPoint c_voxel_size_;
            voxblox::FloatingPoint c_block_size_;
            double c_field_of_view_x_;
            double c_field_of_view_y_;
            int c_res_x_;
            int c_res_y_;

            // testing
            bool p_test_;
            std::vector<double> time_count_;

            // implementations
            std::vector<Eigen::Vector3d>
            (mav_active_3d_planning::RayCaster::*impl_)(Eigen::Vector3d position, Eigen::Quaterniond orientation);

            // Naive Ray casting (timed at 90 +/- 30 ms)
            std::vector <Eigen::Vector3d> implSimple(Eigen::Vector3d position, Eigen::Quaterniond orientation);

            // Cast Rays until they span 1 full voxel, then duplicate them (timed at 55 +/- 25 ms)
            std::vector <Eigen::Vector3d>
            implIterativeRaySampling(Eigen::Vector3d position, Eigen::Quaterniond orientation);
        };

//    } // namespace ray_casters
}  // namespace mav_active_3d_planning
#endif //MAV_ACTIVE_3D_PLANNING_RAY_CASTER_H
