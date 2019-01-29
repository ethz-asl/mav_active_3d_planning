#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/defaults.h"

#include <mav_msgs/eigen_mav_msgs.h>
#include <random>

namespace mav_active_3d_planning {

    // Create random linear segments (leads to non-smooth paths)
    class TGRandomLinear: public TrajectoryGenerator {
    public:
        TGRandomLinear(voxblox::EsdfServer *voxblox_Ptr, bool p_coll_optimistic, double p_coll_radius)
                : TrajectoryGenerator(voxblox_Ptr, p_coll_optimistic, p_coll_radius) {}

        // Overwrite virtual functions
        bool expandSegment(TrajectorySegment &target);
        TrajectorySegment* selectSegment(TrajectorySegment &root);
        bool setParamsFromRos(const ros::NodeHandle &nh);

    protected:
        double p_distance_;         // m
        double p_speed_;            // m/s
        int p_n_segments_;
        int p_max_tries_;
        bool p_planar_;
        double p_uniform_weight_;   // [0-1] % of probability mass that is uniformly distributed
    };

    bool TGRandomLinear::setParamsFromRos(const ros::NodeHandle &nh) {
        nh.param("TG_distance", p_distance_, 1.0);
        nh.param("TG_speed", p_speed_, 0.05);
        nh.param("TG_n_segments", p_n_segments_, 5);
        nh.param("TG_max_tries", p_max_tries_, 1000);
        nh.param("TG_planar", p_planar_, true);
        nh.param("TG_uniform_weight", p_uniform_weight_, 0.2);
    }

    TrajectorySegment* TGRandomLinear::selectSegment(TrajectorySegment &root) {
        if ((double)rand()/RAND_MAX < p_uniform_weight_){
            return defaults::selectRandomLeafUniform(root);
        }
        return defaults::selectRandomLeafWeighted(root);
    }

    bool TGRandomLinear::expandSegment(TrajectorySegment &target) {
        // Create and add new adjacent trajectories to target segment
        int valid_segments = 0;
        double rate = 100;   // Sample trajectory at 100Hz
        Eigen::Vector3d start_pos = target.trajectory.back().position_W;
        srand(time(NULL));
        int counter = 0;

        while (valid_segments < p_n_segments_ && counter < p_max_tries_) {
            counter++;
            // new random target direction
            double yaw = (double) rand() * 2.0 * M_PI / (double) RAND_MAX;
            double theta = (double) rand() * M_PI / (double) RAND_MAX;
            if (p_planar_) { theta = 0.5 * M_PI; }
            Eigen::Vector3d target_pos = start_pos +
                    p_distance_ *Eigen::Vector3d(sin(theta) * cos(yaw), sin(theta) * sin(yaw), cos(theta));
            int n_points = ceil(p_distance_ / p_speed_ * rate);
            bool collided = false;

            // Check collision
            for (int i = 0; i < n_points; ++i) {
                Eigen::Vector3d current_pos = start_pos + (double) i / n_points * (target_pos - start_pos);
                if (!checkCollision(current_pos)) {
                    collided = true;
                    break;
                }
            }
            if (collided) { continue; }

            // Build result
            TrajectorySegment* new_segment = target.spawnChild();
            for (int i = 0; i < n_points; ++i) {
                mav_msgs::EigenTrajectoryPoint trajectory_point;
                trajectory_point.position_W = start_pos + (double) i / n_points * (target_pos - start_pos);
                trajectory_point.setFromYaw(yaw);
                trajectory_point.time_from_start_ns = static_cast<int64_t>((double) i / rate * 1.0e9);
                new_segment->trajectory.push_back(trajectory_point);
            }
            valid_segments++;
        }
        if (counter >= p_max_tries_ && valid_segments==0){
            // No feasible solution: try rotating only
            TrajectorySegment* new_segment = target.spawnChild();
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = start_pos;
            trajectory_point.setFromYaw((double) rand() * 2.0 * M_PI / (double) RAND_MAX);
            trajectory_point.time_from_start_ns = 0;
            new_segment->trajectory.push_back(trajectory_point);
            return false;
        }
        return true;
    }

}  // namespace mav_active_3d_planning