#include "active_3d_planning_core/module/trajectory_evaluator/naive_evaluator.h"

#include <algorithm>

namespace active_3d_planning {
    namespace trajectory_evaluator {

// Factory Registration
        ModuleFactoryRegistry::Registration<NaiveEvaluator>
                NaiveEvaluator::registration("NaiveEvaluator");

        NaiveEvaluator::NaiveEvaluator(PlannerI &planner)
                : SimulatedSensorEvaluator(planner) {}

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
            SimulatedSensorInfo *info =
                    reinterpret_cast<SimulatedSensorInfo *>(traj_in->info.get());
            info->visible_voxels.erase(
                    std::remove_if(info->visible_voxels.begin(), info->visible_voxels.end(),
                                   [this](const Eigen::Vector3d &voxel) {
                                       return planner_.getMap().isObserved(voxel);
                                   }),
                    info->visible_voxels.end());
            traj_in->gain = (double) info->visible_voxels.size();
            return true;
        }

    } // namespace trajectory_evaluator
} // namespace active_3d_planning
