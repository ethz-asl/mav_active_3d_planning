//#include "random_planner.h"
//#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_msgs/Layer.h>
#include <iostream>
#include <random>
#include <cmath>
#include <time.h>

namespace mav_active_planning {

    class RandomPlannerNode {
    public:
        RandomPlannerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~RandomPlannerNode() {}

//        double getMapDistance(const Eigen::Vector3d &position) const;
        void odomCallback(const nav_msgs::Odometry &msg);
        bool runService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber odom_sub_;
        ros::Publisher target_pub_;
        ros::ServiceServer run_srv_;
        voxblox::EsdfServer voxblox_server_;

        Eigen::Vector3d target_position_;
        bool planar_;
        bool running_;
        bool verbose_;
        double last_repub_;
        double target_yaw_;
        double replan_threshold_;
        double replan_new_distance_;
        double replan_speed_;
        double collision_radius_;
        double collision_sweep_step_;

        void requestMovement(const Eigen::Vector3d start, const Eigen::Vector3d goal, const double yaw);
        void randomizeCandidate(Eigen::Vector3d& position, double& yaw, double rand1, double rand2);
        bool checkLinearCollision(const Eigen::Vector3d start, const Eigen::Vector3d goal);
        double getMapDistance(const Eigen::Vector3d& position); //voxblox map distance

    };

    RandomPlannerNode::RandomPlannerNode(const ros::NodeHandle &nh,
                                         const ros::NodeHandle &nh_private)
            : nh_(nh),
              nh_private_(nh_private),
              voxblox_server_(nh_, nh_private_) {
        // initial values and params
        running_ = false;
        replan_threshold_ = 0.05;
        collision_sweep_step_ = 0.05;
        last_repub_ = 0.0;
        target_position_ << 0, 0, 0;
        nh_private_.param("range", replan_new_distance_, 1.0);
        nh_private_.param("verbose", verbose_, false);
        nh_private_.param("speed", replan_speed_, 0.05);
        nh_private_.param("collision_radius", collision_radius_, 0.35);
        nh_private_.param("planar", planar_, true);

        // Subscribers and publishers
        target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        odom_sub_ = nh_.subscribe("odometry", 1, &RandomPlannerNode::odomCallback, this);
        run_srv_ = nh_private_.advertiseService("toggle_running", &RandomPlannerNode::runService, this);

        // setup voxblox collision checking and wait for first map
        ros::topic::waitForMessage<voxblox_msgs::Layer>("esdf_map_in", nh_private_);
        voxblox_server_.setTraversabilityRadius(collision_radius_);
        voxblox_server_.publishTraversable();
    }

    void RandomPlannerNode::odomCallback(const nav_msgs::Odometry &msg){
        if (!running_){ return;}
        geometry_msgs::Point pos = msg.pose.pose.position;
        Eigen::Vector3d position(pos.x, pos.y, pos.z);
        geometry_msgs::Vector3 vel = msg.twist.twist.linear;
        Eigen::Vector3d velocity(vel.x, vel.y, vel.z);
        double distance = (target_position_ - position).squaredNorm();
        if (distance < replan_threshold_)
        {
            // Find a new target position
            srand(time(NULL));
            bool path_found = false;
            int count = 0;
            Eigen::Vector3d candidate_pos;
            double candidate_yaw = 0;
            while (!path_found && count < 10000)
            {
                candidate_pos = position;
                randomizeCandidate(candidate_pos, candidate_yaw, (double)rand(), (double)rand());
                path_found = checkLinearCollision(position, candidate_pos);
                count++;
            }
            if (verbose_) {
                if (path_found) {
                    ROS_INFO("Random Planner: New feasible target [%.2f %.2f %.2f] found after %i iterations.",
                    candidate_pos[0], candidate_pos[1], candidate_pos[2], count);
                } else {
                    ROS_INFO("Random Planner: Maximum iterations reached, publishing random target [%.2f %.2f %.2f].",
                             candidate_pos[0], candidate_pos[1], candidate_pos[2]);
                }
            }
            requestMovement(position, candidate_pos, candidate_yaw);
        }
        else if (velocity.squaredNorm() <= replan_speed_/5 && msg.header.stamp.toSec()-last_repub_ > 2.0)
        {
            if (verbose_){
                ROS_INFO("Random Planner: Republishing target due to inactivity (speed=%.5f).", velocity.squaredNorm());
            }
            requestMovement(position, target_position_, target_yaw_);
        }
    }

    void RandomPlannerNode::requestMovement(const Eigen::Vector3d start, const Eigen::Vector3d goal, const double yaw) {
        target_position_ = goal;
        target_yaw_ = yaw;
        trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
        msg->header.stamp = ros::Time::now();
        msg->joint_names.push_back("base_link");
        double total_time = (goal-start).squaredNorm()/replan_speed_;
        int n_points = floor(total_time*10);  // 10Hz points
        msg->points.resize(n_points+1);
        for (int i = 0; i <= n_points; ++i) {
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = start + (double)i / n_points * (goal-start);
            trajectory_point.setFromYaw(yaw);
            trajectory_point.time_from_start_ns = static_cast<int64_t>((double)i / n_points * total_time * 1.0e9);
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
        }
        target_pub_.publish(msg);
        last_repub_ = msg->header.stamp.toSec();
        voxblox_server_.publishTraversable();
    }

    void RandomPlannerNode::randomizeCandidate(Eigen::Vector3d& position, double& yaw, double rand1, double rand2) {
        // spherical or planar random walk, with position being the current pos.
        yaw =  rand1 * 2.0 * M_PI / (double)RAND_MAX;
        double theta = rand2 * M_PI / (double)RAND_MAX;
        if (planar_){theta = 0.5 * M_PI;}
        position[0] += replan_new_distance_ * sin(theta) * cos(yaw);
        position[1] += replan_new_distance_ * sin(theta) * sin(yaw);
        position[2] += replan_new_distance_ * cos(theta);
    }

    bool RandomPlannerNode::checkLinearCollision(const Eigen::Vector3d start, const Eigen::Vector3d goal) {
        // Sweep linear path and check with voxblox
        Eigen::Vector3d loc;
        double distance;
        int n_steps = floor((goal - start).squaredNorm()/collision_sweep_step_);
        for (int i=1; i< n_steps; i++)
        {
            loc = start + (double)i * collision_sweep_step_ *(goal-start);
            distance = getMapDistance(loc);
            if (distance <= collision_radius_) {return false;}
        }
        return true;
    }

    double RandomPlannerNode::getMapDistance(const Eigen::Vector3d& position) {
        if (!voxblox_server_.getEsdfMapPtr())
        {
            return 0.0;
        }
        double distance = 0.0;
        if (!voxblox_server_.getEsdfMapPtr()->getDistanceAtPosition(position, &distance))
        {
            return 0.0;
        }
        return distance;
    }

    bool RandomPlannerNode::runService(std_srvs::SetBool::Request  &req,
                    std_srvs::SetBool::Response &res) {
        running_ = req.data;
        res.success = true;
        if (running_)
        {
            ROS_INFO("Random Planner: Started planning.");

        } else
        {
            ROS_INFO("Random Planner: Stopped planning.");
        }
        return true;
    }

} // namespace mav_active_planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_planner");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    mav_active_planning::RandomPlannerNode planner_node(nh, nh_private);

    ROS_INFO("Initialized random_planner node.");
    ros::spin();
}