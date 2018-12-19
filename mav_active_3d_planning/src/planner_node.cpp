#include "mav_active_3d_planning/trajectory_segment.h"
#include "tg_random_linear.cpp"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <std_srvs/SetBool.h>
#include <voxblox_ros/esdf_server.h>
#include <voxblox_msgs/Layer.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <random>
#include <algorithm>

namespace mav_active_3d_planning {

    class PlannerNode {
    public:
        PlannerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~PlannerNode() {}

        // ros callbacks
        void odomCallback(const nav_msgs::Odometry &msg);
        bool runService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    protected:
        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber odom_sub_;
        ros::Publisher target_pub_;
        ros::Publisher trajectory_vis_pub_;
        ros::ServiceServer run_srv_;

        // members
        voxblox::EsdfServer voxblox_server_;
        TrajectoryGenerator* trajectory_generator_;

        // variables
        TrajectorySegment current_segment_;
        bool running_;
        Eigen::Vector3d target_position_;

        // params
        double p_replan_threshold_;
        std::string p_TG_name_;
        bool p_coll_optimistic_;
        double p_coll_radius_;
        bool p_verbose_;

        // methods
        void setParamsFromRos();
        void replan();
        void requestMovement(const TrajectorySegment trajectory);
        void publishTrajectoryVisualization(std::vector<TrajectorySegment> trajectories);
    };

    PlannerNode::PlannerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
            : nh_(nh),
              nh_private_(nh_private),
              voxblox_server_(nh_, nh_private_) {

        // initial values and params
        setParamsFromRos();
        p_replan_threshold_ = 0.05;
        target_position_ = Eigen::Vector3d(0, 0, 0);
        running_ = false;

        // Subscribers and publishers
        target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory_visualization", 100);
        odom_sub_ = nh_.subscribe("odometry", 1, &PlannerNode::odomCallback, this);

        // Setup trajectory generator
        if (p_TG_name_ == "RandomLinear") {
            trajectory_generator_ = new TGRandomLinear(&voxblox_server_, p_coll_optimistic_, p_coll_radius_);
        } else {
            ROS_ERROR("Unknown trajectory generator '%s'.", p_TG_name_.c_str());
        }
        trajectory_generator_->setParamsFromRos(nh_private_);

        // Setup initial trajectory Segment
        current_segment_ = TrajectorySegment();
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = Eigen::Vector3d(0, 0, 0);
        trajectory_point.setFromYaw(0);
        current_segment_.trajectory.push_back(trajectory_point);

        // setup voxblox collision checking and wait for first map
        ros::topic::waitForMessage<voxblox_msgs::Layer>("esdf_map_in", nh_private_);
        voxblox_server_.setTraversabilityRadius(p_coll_radius_);
        voxblox_server_.publishTraversable();

        // Finish
        run_srv_ = nh_private_.advertiseService("toggle_running", &PlannerNode::runService, this);

    }

    void PlannerNode::setParamsFromRos() {
        nh_private_.param("verbose", p_verbose_, false);
        nh_private_.param("trajectory_generator", p_TG_name_, std::string("RandomLinear"));
        nh_private_.param("collision_optimistic", p_coll_optimistic_, false);
        nh_private_.param("collision_radius", p_coll_radius_, 0.35);
    }

    void PlannerNode::odomCallback(const nav_msgs::Odometry &msg) {
        // Currently a testing function that does replanning once the end point of a trajectory is reached
        if (!running_) { return; }
        geometry_msgs::Point pos = msg.pose.pose.position;
        Eigen::Vector3d position(pos.x, pos.y, pos.z);
//        geometry_msgs::Vector3 vel = msg.twist.twist.linear;
//        Eigen::Vector3d velocity(vel.x, vel.y, vel.z);
        double distance = (target_position_ - position).squaredNorm();
        if (distance < p_replan_threshold_) {
            replan();
        }
    }

    void PlannerNode::replan() {
        std::cout << "Replanning!" << std::endl;

        // Get new trajectories
        std::vector<TrajectorySegment> result;  // stores trajectories
        trajectory_generator_->expandSegment(&result, current_segment_);

        // Compute information gain/cost (currently random test)
        for (int i=0; i<result.size(); ++i) {
            result[i].cost = (double) rand() / RAND_MAX;
        }

        // Select best trajectory   (Or any tree exploring algo, depending on type)
        current_segment_ = *std::max_element(result.begin(), result.end(), TrajectorySegment::compare);

        // publish path
        requestMovement(current_segment_);

        // Visualization
        voxblox_server_.publishTraversable();
        publishTrajectoryVisualization(result);
    }

    void PlannerNode::requestMovement(const TrajectorySegment req) {
        std::cout << "Publishing trajectory" << std::endl;
        trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
        msg->header.stamp = ros::Time::now();
        msg->joint_names.push_back("base_link");
        int n_points = req.trajectory.size();
        msg->points.resize(n_points);
        for (int i = 0; i < n_points; ++i) {
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(req.trajectory[i], &msg->points[i]);
        }
        target_pub_.publish(msg);
        target_position_ = req.trajectory.back().position_W;
    }

    void PlannerNode::publishTrajectoryVisualization(std::vector<TrajectorySegment> trajectories) {
        double max_cost = std::max_element(trajectories.begin(), trajectories.end(), TrajectorySegment::compare)->cost;
        double min_cost = std::min_element(trajectories.begin(), trajectories.end(), TrajectorySegment::compare)->cost;
        for (int i = 0; i < trajectories.size(); ++i) {
            // Setup marker message
            visualization_msgs::Marker msg;
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            msg.pose.orientation.w = 1.0;
            msg.type = visualization_msgs::Marker::POINTS;
            msg.id = i;
            msg.ns = "candidate_trajectories";
            msg.scale.x = 0.03;
            msg.scale.y = 0.03;

            // Color according to relative cost
            double frac = (trajectories[i].cost - min_cost) / (max_cost - min_cost);
            msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
            msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
            msg.color.b = 0.0;
            msg.color.a = 0.2;

            // points
            for (int j = 0; j < trajectories[i].trajectory.size(); ++j) {
                geometry_msgs::Point point;
                point.x = trajectories[i].trajectory[j].position_W[0];
                point.y = trajectories[i].trajectory[j].position_W[1];
                point.z = trajectories[i].trajectory[j].position_W[2];
                msg.points.push_back(point);
            }
            trajectory_vis_pub_.publish(msg);
        }
    }

    bool PlannerNode::runService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        running_ = req.data;
        res.success = true;
        if (running_) {
            ROS_INFO("Started planning.");

        } else {
            ROS_INFO("Stopped planning.");
        }
        return true;
    }

} // namespace mav_active_3d_planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "active_3d_planner");

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::PlannerNode planner_node(nh, nh_private);

    ROS_INFO("Initialized active_3d_planner node.");
    ros::spin();
}