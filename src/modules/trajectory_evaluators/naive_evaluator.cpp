#include "mav_active_3d_planning/modules/trajectory_evaluators/naive_evaluator.h"

#include <algorithm>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Factory Registration
        ModuleFactory::Registration<NaiveEvaluator> NaiveEvaluator::registration("NaiveEvaluator");

        void NaiveEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            // setup parent
            SimulatedSensorEvaluator::setupFromParamMap(param_map);
        }

        bool NaiveEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            if (!traj_in->info) {
                traj_in->gain = 0.0;
                return false;
            }
            // remove all already observed voxels, count number of new voxels
            SimulatedSensorInfo *info = dynamic_cast<SimulatedSensorInfo *>(traj_in->info.get());
            info->visible_voxels.erase(
                    std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                                   [this](const Eigen::Vector3d &voxel) {
                                       return voxblox_ptr_->getEsdfMapPtr()->isObserved(voxel);
                                   }), info->visible_voxels.end());
            traj_in->gain = (double) info->visible_voxels.size();
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
