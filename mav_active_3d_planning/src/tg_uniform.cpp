#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <random>

namespace mav_active_3d_planning {

    class TGUniform: public TrajectoryGenerator {
    public:
        TGUniform(voxblox::EsdfServer *voxblox_Ptr, bool p_coll_optimistic, double p_coll_radius)
                : TrajectoryGenerator(voxblox_Ptr, p_coll_optimistic, p_coll_radius) {}

        // Overwrite virtual functions
        bool expandSegment(TrajectorySegment &target);
        TrajectorySegment* selectSegment(TrajectorySegment &root);
        bool setParamsFromRos(const ros::NodeHandle &nh);

    protected:
        // parameters
        double p_distance_;         // m
        double p_velocity_;         // m/s
        double p_yaw_rate_max_;     // rad/s
        int p_n_segments_;
        bool p_planar_;
        double p_uniform_weight_;   // [0-1] % of probability mass that is uniformly distributed
    };

    bool TGUniform::setParamsFromRos(const ros::NodeHandle &nh) {
        nh.param("TG_distance", p_distance_, 1.0);
        nh.param("TG_velocity", p_velocity_, 0.05);
        nh.param("TG_yaw_rate_max", p_yaw_rate_max_, 0.05);
        nh.param("TG_n_segments", p_n_segments_, 5);
        nh.param("TG_planar", p_planar_, true);
        nh.param("TG_uniform_weight", p_uniform_weight_, 0.2);
    }

    TrajectorySegment* TGUniform::selectSegment(TrajectorySegment &root) {
        return defaults::selectRandomLeafWeighted(root, p_uniform_weight_);
    }

    bool TGUniform::expandSegment(TrajectorySegment &target) {
        // Create and add new adjacent trajectories to target segment
        const double rate = 10;   // Sample trajectory at 10Hz
        srand(time(NULL));
        int valid_segments = 0;
        TrajectorySegment* new_segment = target.spawnChild();

        for (int i=0; i < p_n_segments_; ++i) {
            // Initialization
            new_segment->trajectory.clear();
            Eigen::Vector3d current_pos = target.trajectory.back().position_W;
            double yaw_rate = ((double) i - (double) p_n_segments_ / 2.0 + 0.5) * p_yaw_rate_max_;
            double current_yaw = target.trajectory.back().getYaw();
            double current_distance = 0.0;
            double current_time = 0.0;
            bool collided = false;

            while (current_distance < p_distance_){
                // Advance trajectory for every timestep
                current_yaw += yaw_rate / rate;
                current_distance += p_velocity_ / rate;
                current_time += 1.0 / rate;
                current_pos += p_velocity_ / rate * Eigen::Vector3d(cos(current_yaw), sin(current_yaw), 0.0);
                if (!checkCollision(current_pos)) {
                    collided = true;
                    break;
                }
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = current_pos;
                trajectory_point.setFromYaw(current_yaw);
                trajectory_point.time_from_start_ns = static_cast<int64_t>(current_time*1.0e9);
                new_segment->trajectory.push_back(trajectory_point);
            }
            if (!collided){
                valid_segments++;
                new_segment = target.spawnChild();
            }
        }
        if (valid_segments == 0){
            // No feasible solution: try rotating only
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = target.trajectory.back().position_W;
            trajectory_point.setFromYaw(target.trajectory.back().getYaw() + 0.03);
            trajectory_point.time_from_start_ns = 0;
            new_segment->trajectory.clear();
            new_segment->trajectory.push_back(trajectory_point);
            return false;
        } else {
            target.children.pop_back();
            return true;
        }
    }

}  // namespace mav_active_3d_planning