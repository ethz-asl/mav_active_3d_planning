#include "mav_active_3d_planning/modules/trajectory_evaluators/voxel_type_evaluator.h"

#include <algorithm>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Factory Registration
        ModuleFactory::Registration<VoxelTypeEvaluator> VoxelTypeEvaluator::registration("VoxelTypeEvaluator");

        void VoxelTypeEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            // setup parent
            SimulatedSensorEvaluator::setupFromParamMap(param_map);

            // params
            setParam<double>(param_map, "gain_unknown", &p_gain_unknown_, 1.0);
            setParam<double>(param_map, "gain_occupied", &p_gain_occupied_, 0.0);
            setParam<double>(param_map, "gain_free", &p_gain_free_, 0.0);
            setParam<double>(param_map, "gain_unknown_outer", &p_gain_unknown_outer_, 0.0);
            setParam<double>(param_map, "gain_occupied_outer", &p_gain_occupied_outer_, 0.0);
            setParam<double>(param_map, "gain_free_outer", &p_gain_free_outer_, 0.0);

            // outer volume (if not specified will not be counted)
            std::string ns = (*param_map)["param_namespace"];
            std::string outer_volume_args;
            setParam<std::string>(param_map, "outer_volume_args", &outer_volume_args, ns + "/outer_volume");
            outer_volume_ = ModuleFactory::Instance()->createModule<defaults::BoundingVolume>(outer_volume_args,
                                                                                              verbose_modules_);

            // constants
            c_min_gain_ = std::min({p_gain_unknown_, p_gain_occupied_, p_gain_free_, p_gain_unknown_outer_,
                                    p_gain_occupied_outer_, p_gain_free_outer_});
            c_max_gain_ = std::max({p_gain_unknown_, p_gain_occupied_, p_gain_free_, p_gain_unknown_outer_,
                                    p_gain_occupied_outer_, p_gain_free_outer_});
        }

        bool VoxelTypeEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            traj_in->gain = 0.0;
            if (!traj_in->info) { return false; }
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(traj_in->info.get());
            for (int i = 0; i < info->visible_voxels.size(); ++i) {
                double distance = 0.0;
                if (bounding_volume_->contains(info->visible_voxels[i])) {
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(info->visible_voxels[i], &distance)) {
                        if (distance < 0.0) {
                            traj_in->gain += p_gain_occupied_;
                        } else {
                            traj_in->gain += p_gain_free_;
                        }
                    } else {
                        traj_in->gain += p_gain_unknown_;
                    }
                } else {
                    if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(info->visible_voxels[i],
                                                                             &distance)) {
                        if (distance < 0.0) {
                            traj_in->gain += p_gain_occupied_outer_;
                        } else {
                            traj_in->gain += p_gain_free_outer_;
                        }
                    } else {
                        traj_in->gain += p_gain_unknown_outer_;
                    }
                }
            }
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
