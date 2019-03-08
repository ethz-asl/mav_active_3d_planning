#include "mav_active_3d_planning/modules/trajectory_evaluators/cost_computers/default_cost_computers.h"

namespace mav_active_3d_planning {
    namespace cost_computers {

        // SegmentTime
        bool SegmentTime::computeCost(TrajectorySegment *traj_in) {
            if (traj_in->trajectory.size() < 2) {
                traj_in->cost = 0.0;
                return false;
            }
            traj_in->cost = static_cast<double>(traj_in->trajectory.back().time_from_start_ns -
                                                traj_in->trajectory.front().time_from_start_ns) * 1.0e-9;
            return true;
        }

        // SegmentLength
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
            return true;
        }

    } // namespace cost_computers
} // namepsace mav_active_3d_planning
