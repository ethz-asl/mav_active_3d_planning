#include "mav_active_3d_planning/modules/trajectory_evaluators/cost_computers/default_cost_computers.h"

namespace mav_active_3d_planning {
    namespace cost_computers {

        // SegmentTime
        ModuleFactory::Registration<SegmentTime> SegmentTime::registration("SegmentTime");

        void SegmentTime::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
        }

        bool SegmentTime::computeCost(TrajectorySegment *traj_in) {
            if (traj_in->trajectory.size() < 2) {
                traj_in->cost = 0.0;
                return false;
            }
            traj_in->cost = static_cast<double>(traj_in->trajectory.back().time_from_start_ns -
                                                traj_in->trajectory.front().time_from_start_ns) * 1.0e-9;
            if (p_accumulate_ && traj_in->parent) {
                traj_in->cost += traj_in->parent->cost;
            }
            return true;
        }

        // SegmentLength
        ModuleFactory::Registration<SegmentLength> SegmentLength::registration("SegmentLength");

        void SegmentLength::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "accumulate", &p_accumulate_, false);
        }

        bool SegmentLength::computeCost(TrajectorySegment *traj_in) {
            if (traj_in->trajectory.size() < 2) {
                traj_in->cost = 0.0;
                return false;
            }
            // Linear path interpolation
            double distance = 0.0;
            Eigen::Vector3d last_point = traj_in->trajectory[0].position_W;
            for (int i = 1; i < traj_in->trajectory.size(); ++i) {
                distance += (traj_in->trajectory[i].position_W - last_point).norm();
                last_point = traj_in->trajectory[i].position_W;
            }
            traj_in->cost = distance;
            if (p_accumulate_ && traj_in->parent) {
                traj_in->cost += traj_in->parent->cost;
            }
            return true;
        }

        // NoCost
        ModuleFactory::Registration<NoCost> NoCost::registration("NoCost");

        void NoCost::setupFromParamMap(Module::ParamMap *param_map) {}

        bool NoCost::computeCost(TrajectorySegment *traj_in) {
            traj_in -> cost = 0.0;
            if (traj_in->trajectory.size() < 2) {
                return false;
            }
            return true;
        }

    } // namespace cost_computers
} // namepsace mav_active_3d_planning
