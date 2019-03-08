#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/defaults.h"
#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/ray_caster.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Naive just counts the number of yet unobserved visible voxels.
        class Naive: public TrajectoryEvaluator {
        public:
            Naive(std::shared_ptr<voxblox::EsdfServer> voxblox_ptr, std::string param_ns);

            // Overwrite virtual functions
            bool computeGain(TrajectorySegment *traj_in);

        protected:
            friend ModuleFactory;

            Naive() {}

            void setupFromParamMap(ParamMap *param_map);

            // members
            RayCaster ray_caster_;

            // parameters
            bool p_clear_from_parents_;
        };

//        Naive::Naive(std::shared_ptr<voxblox::EsdfServer> voxblox_ptr, std::string param_ns)
//            : TrajectoryEvaluator(voxblox_ptr, param_ns),
//              ray_caster_(voxblox_ptr, param_ns) {
//            // params
//            ros::param::param<bool>(param_ns + "/clear_from_parents", p_clear_from_parents_, false);
//        }

        void Naive::setupFromParamMap(ParamMap *param_map){
//                setParam<double>(param_map, "cost_weight", &cost_weight_, 1.0);
        }

        bool Naive::computeGain(TrajectorySegment *traj_in) {
            std::vector <Eigen::Vector3d> new_voxels = ray_caster_.getVisibleVoxelsFromTrajectory(traj_in);

            // Check for interesting bounding box
            if (bounding_volume_.is_setup){
                new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(),
                                                [this](const Eigen::Vector3d& voxel)
                                                { return !bounding_volume_.contains(voxel); }),
                                 new_voxels.end());
            }

            // Remove already known voxels
            new_voxels.erase(std::remove_if(new_voxels.begin(), new_voxels.end(), [this](const Eigen::Vector3d& voxel) {
                return voxblox_ptr_->getEsdfMapPtr()->isObserved(voxel); } ), new_voxels.end());
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

            // Set gain (#new voxels)
            traj_in->gain = (double)traj_in->info.size();
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning