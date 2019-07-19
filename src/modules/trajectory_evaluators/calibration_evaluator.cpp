#include "mav_active_3d_planning/modules/trajectory_evaluators/calibration_evaluator.h"

#include <algorithm>

#include <geometry_msgs/PoseStamped.h>

#include <mav_active_3d_planning/EvaluateTrajectoryService.h>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // Factory Registration
        ModuleFactory::Registration<CalibrationEvaluator> CalibrationEvaluator::registration("CalibrationEvaluator");

        void CalibrationEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            // setup parent
            SimulatedSensorEvaluator::setupFromParamMap(param_map);
            setParam<std::string>(param_map, "evaluation_service_name", &evaluation_service_name, "");
            evaluation_client = node_handle.serviceClient<mav_active_3d_planning::EvaluateTrajectoryService>(evaluation_service_name);
        }

        bool CalibrationEvaluator::computeGainFromVisibleVoxels(TrajectorySegment *traj_in) {
            if (!traj_in->info) {
                traj_in->gain = 0.0;
                return false;
            }
            // call service
            std::vector<geometry_msgs::PoseStamped> poses_stamped;
            for(auto&& eigen_trajectory_point_vector : traj_in->trajectory){
                geometry_msgs::PoseStamped p;
                if(eigen_trajectory_point_vector.timestamp_ns >= 0){
                    p.header.stamp.nsec = eigen_trajectory_point_vector.timestamp_ns;
                } else {
                    p.header.stamp.nsec = eigen_trajectory_point_vector.time_from_start_ns;
                }
                p.pose.position.x = eigen_trajectory_point_vector.position_W.x();
                p.pose.position.y = eigen_trajectory_point_vector.position_W.y();
                p.pose.position.z = eigen_trajectory_point_vector.position_W.z();

                p.pose.orientation.x = eigen_trajectory_point_vector.orientation_W_B.x();
                p.pose.orientation.y = eigen_trajectory_point_vector.orientation_W_B.y();
                p.pose.orientation.z = eigen_trajectory_point_vector.orientation_W_B.z();
                p.pose.orientation.w = eigen_trajectory_point_vector.orientation_W_B.w();

                poses_stamped.push_back(p);
            }
            mav_active_3d_planning::EvaluateTrajectoryService srv;
            srv.request.posesStamped = poses_stamped;
            if(evaluation_client.call(srv)) {
                traj_in->gain = srv.response.quality;
                return true;
            }
            else {
                traj_in->gain = 0.0;
                return false;
            }
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
