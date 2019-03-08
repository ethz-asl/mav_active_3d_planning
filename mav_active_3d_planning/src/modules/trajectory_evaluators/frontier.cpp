#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/defaults.h"
#include "mav_active_3d_planning/ray_caster.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        class Frontier: public TrajectoryEvaluator {
        public:
            Frontier(std::shared_ptr<voxblox::EsdfServer> voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool computeGain(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            Frontier() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // members
            RayCaster ray_caster_;

            // parameters
            double p_sampling_time_;
            bool p_clear_from_parents_;
        };

//        Frontier::Frontier(std::shared_ptr<voxblox::EsdfServer> voxblox_ptr, std::string param_ns)
//            : TrajectoryEvaluator(voxblox_ptr, param_ns),
//              ray_caster_(voxblox_ptr, param_ns) {
//            // params
//            ros::param::param<bool>(param_ns + "/clear_from_parents", p_clear_from_parents_, false);
//        }

        void Frontier::setupFromParamMap(Module::ParamMap *param_map){
//            setParam<bool>(param_map, "cost_weight", &cost_weight_, bool);
        }

        bool Frontier::computeGain(TrajectorySegment *traj_in) {
            std::vector <Eigen::Vector3d> new_voxels = ray_caster_.getVisibleVoxelsFromTrajectory(traj_in);


            // Check for interesting bounding box
            if (bounding_volume_.is_setup){
                new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                        [this](const Eigen::Vector3d& voxel)
                        { return !bounding_volume_.contains(voxel); }),
                                new_voxels.end());
            }

            // Remove already observed voxels
            voxblox::EsdfMap* esdf_map = voxblox_ptr_->getEsdfMapPtr().get();
            new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                    [esdf_map](const Eigen::Vector3d& voxel) { return esdf_map->isObserved(voxel); } ),
                            new_voxels.end());

            // Remove non-unique voxels
            new_voxels.erase( std::unique( new_voxels.begin(), new_voxels.end()), new_voxels.end() );

            // Remove non-frontier voxels
            double voxel_size = static_cast<double>(voxblox_ptr_->getEsdfMapPtr()->voxel_size());
            new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                    [esdf_map, voxel_size](const Eigen::Vector3d& voxel) {
                        // Check all neighboring voxels
                        double distance;
                        if (esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(voxel_size, 0, 0), &distance)){
                            if (distance < voxel_size){ return false;}
                        }
                        if (esdf_map->getDistanceAtPosition(voxel - Eigen::Vector3d(voxel_size, 0, 0), &distance)){
                            if (distance < voxel_size){ return false;}
                        }
                        if (esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, voxel_size, 0), &distance)){
                            if (distance < voxel_size){ return false;}
                        }
                        if (esdf_map->getDistanceAtPosition(voxel - Eigen::Vector3d(0, voxel_size, 0), &distance)){
                            if (distance < voxel_size){ return false;}
                        }
                        if (esdf_map->getDistanceAtPosition(voxel + Eigen::Vector3d(0, 0, voxel_size), &distance)){
                            if (distance < voxel_size){ return false;}
                        }
                        if (esdf_map->getDistanceAtPosition(voxel - Eigen::Vector3d(0, 0, voxel_size), &distance)){
                            if (distance < voxel_size){ return false;}
                        }
                        return true; } ), new_voxels.end());

            traj_in->info = new_voxels;

            // Remove voxels previously seen by parent (this is quite expensive)
            if(p_clear_from_parents_){
                TrajectorySegment* previous = traj_in->parent;
                while (previous){
                    std::vector <Eigen::Vector3d> old_indices = previous->info;
                    traj_in->info.erase(
                            std::remove_if(
                                    traj_in->info.begin(),
                                    traj_in->info.end(),
                                    [&old_indices](const Eigen::Vector3d& global_index)
                                    {
                                        auto it = std::find(old_indices.begin(), old_indices.end(), global_index);
                                        return (it != old_indices.end());
                                    }),
                            traj_in->info.end());
                    previous = previous->parent;
                }
            }

            // Set gain (#frontier voxels)
            traj_in->gain = (double)traj_in->info.size();
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning