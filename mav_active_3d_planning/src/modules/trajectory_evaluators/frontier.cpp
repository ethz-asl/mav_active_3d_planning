#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        class Frontier: public TrajectoryEvaluator {
        public:
            Frontier(voxblox::EsdfServer *voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool computeGain(TrajectorySegment &traj_in);

        protected:
            // members
            RayCaster ray_caster_;

            // parameters
            double p_sampling_time_;
            bool p_clear_from_parents_;
        };

        Frontier::Frontier(voxblox::EsdfServer *voxblox_ptr, std::string param_ns)
            : TrajectoryEvaluator(voxblox_ptr, param_ns),
              ray_caster_(voxblox_ptr, param_ns) {
            // params
            ros::param::param<double>(param_ns + "/sampling_time", p_sampling_time_, 0.0);
            ros::param::param<bool>(param_ns + "/clear_from_parents", p_clear_from_parents_, false);
        }

        bool Frontier::computeGain(TrajectorySegment &traj_in) {
            traj_in.info.clear();
            std::vector<int> indices = defaults::samplePointsFromSegment(traj_in, p_sampling_time_);
            for (int i=0; i < indices.size(); ++i ){
                std::vector <Eigen::Vector3d> new_voxels = ray_caster_.getVisibleVoxels(
                        traj_in.trajectory[indices[i]].position_W, traj_in.trajectory[indices[i]].orientation_W_B);

                // Check for interesting bounding box and already known voxels
                if (bounding_volume_.is_setup){
                    new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                            [this](const Eigen::Vector3d& voxel)
                            { return !bounding_volume_.contains(voxel); }),
                                    new_voxels.end());
                }
                voxblox::EsdfMap* esdf_map = voxblox_ptr_->getEsdfMapPtr().get();
                new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(), [esdf_map](const Eigen::Vector3d& voxel) {
                    return esdf_map->isObserved(voxel); } ), new_voxels.end());

                // Remove non-frontier voxels
                double voxel_size = static_cast<double>(voxblox_ptr_->getEsdfMapPtr()->voxel_size());
                double *distance;
                new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                        [esdf_map, voxel_size, distance](const Eigen::Vector3d& voxel) {
                            // Check all neighboring voxels
                            esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(voxel_size, 0, 0), distance);
                            if (*distance > voxel_size){ return true;}
                            esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(-voxel_size, 0, 0), distance);
                            if (*distance > voxel_size){ return true;}
                            esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, voxel_size, 0), distance);
                            if (*distance > voxel_size){ return true;}
                            esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, -voxel_size, 0), distance);
                            if (*distance > voxel_size){ return true;}
                            esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, 0, voxel_size), distance);
                            if (*distance > voxel_size){ return true;}
                            esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, 0, -voxel_size), distance);
                            if (*distance > voxel_size){ return true;}
                            return false; } ), new_voxels.end());
                traj_in.info.insert(traj_in.info.begin(), new_voxels.begin(), new_voxels.end());
            }

            // Remove non-unique voxels
            traj_in.info.erase( std::unique( traj_in.info.begin(), traj_in.info.end()), traj_in.info.end() );

            // Remove voxels previously seen by parent (this is quite expensive)
    //        if(p_clear_from_parents_){
    //            TrajectorySegment* previous = traj_in.parent;
    //            while (previous){
    //                std::vector <Eigen::Vector3d> old_indices = previous->info;
    //                traj_in.info.erase(
    //                        std::remove_if(
    //                                traj_in.info.begin(),
    //                                traj_in.info.end(),
    //                                [&old_indices](const Eigen::Vector3d& global_index)
    //                                {
    //                                    auto it = std::find(old_indices.begin(), old_indices.end(), global_index);
    //                                    return (it != old_indices.end());
    //                                }),
    //                        traj_in.info.end());
    //                previous = previous->parent;
    //            }
    //        }

            // Set gain (#new voxels)
            traj_in.gain = (double)traj_in.info.size();
            return true;
        }
    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning