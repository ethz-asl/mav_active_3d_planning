#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/param.h>

#include <vector>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Adds a constant gain for dependent on the type of newly observed voxel
        class VoxelType: public TrajectoryEvaluator {
        public:
            VoxelType(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool computeGain(TrajectorySegment &traj_in);

        protected:
            // members
            RayCaster ray_caster_;
            defaults::BoundingVolume* outer_volume_;

            // parameters
            double p_sampling_time_;
            bool p_clear_from_parents_;
            double p_gain_unknown_;
            double p_gain_unknown_outer_;
            double p_gain_occupied_;
            double p_gain_occupied_outer_;
            double p_gain_free_;
            double p_gain_free_outer_;
        };

        VoxelType::VoxelType(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
            : TrajectoryEvaluator(voxblox_ptr, param_ns),
              ray_caster_(voxblox_ptr, param_ns) {
            // params
            ros::param::param<bool>(param_ns + "/clear_from_parents", p_clear_from_parents_, false);
            ros::param::param<double>(param_ns + "/gain_unknown", p_gain_unknown_, 1.0);
            ros::param::param<double>(param_ns + "/gain_occupied", p_gain_occupied_, 0.0);
            ros::param::param<double>(param_ns + "/gain_free", p_gain_free_, 0.0);
            ros::param::param<double>(param_ns + "/gain_unknown_outer", p_gain_unknown_outer_, 0.0);
            ros::param::param<double>(param_ns + "/gain_occupied_outer", p_gain_occupied_outer_, 0.0);
            ros::param::param<double>(param_ns + "/gain_free_outer", p_gain_free_outer_, 0.0);

            // Setup outer volume (if its not set the everything will be included)
            std::string outer_volume_ns;
            ros::param::param<std::string>(param_ns + "/outer_volume_namespace", outer_volume_ns, std::string(""));
            outer_volume_ = new defaults::BoundingVolume(outer_volume_ns);
        }

        bool VoxelType::computeGain(TrajectorySegment &traj_in) {
            std::vector <Eigen::Vector3d> new_voxels = ray_caster_.getVisibleVoxelsFromTrajectory(&traj_in);

            // Check for outer bounding box
            if (outer_volume_->is_setup){
                new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                                                [this](const Eigen::Vector3d& voxel)
                                                { return !outer_volume_->contains(voxel); }),
                                 new_voxels.end());
            }

            // Remove non-unique voxels
            new_voxels.erase( std::unique( new_voxels.begin(), new_voxels.end()), new_voxels.end() );
            traj_in.info = new_voxels;

            // Remove voxels previously seen by parent (this is quite expensive)
            if(p_clear_from_parents_){
                TrajectorySegment* previous = traj_in.parent;
                while (previous){
                    std::vector <Eigen::Vector3d> old_indices = previous->info;
                    traj_in.info.erase(
                            std::remove_if(
                                    traj_in.info.begin(),
                                    traj_in.info.end(),
                                    [&old_indices](const Eigen::Vector3d& global_index)
                                    {
                                        auto it = std::find(old_indices.begin(), old_indices.end(), global_index);
                                        return (it != old_indices.end());
                                    }),
                            traj_in.info.end());
                    previous = previous->parent;
                }
            }

            // Count the different voxel types for gain
            traj_in.gain = 0.0;
            for (int i = 0; i < traj_in.info.size(); ++i) {
                double distance = 0.0;
                if (bounding_volume_.contains(traj_in.info[i])) {
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(traj_in.info[i], &distance)) {
                        if (distance < 0.0){
                            traj_in.gain += p_gain_occupied_;
                        } else {
                            traj_in.gain += p_gain_free_;
                        }
                    } else {
                        traj_in.gain += p_gain_unknown_;
                    }
                } else {
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(traj_in.info[i], &distance)) {
                        if (distance < 0.0){
                            traj_in.gain += p_gain_occupied_outer_;
                        } else {
                            traj_in.gain += p_gain_free_outer_;
                        }
                    } else {
                        traj_in.gain += p_gain_unknown_outer_;
                    }
                }
            }
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning