#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/trajectory_generator.h"
#include "tg_random_linear.cpp"
#include "tg_mav_trajectory_generation.cpp"
#include "tg_uniform.cpp"
#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "te_naive.cpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>

#include <voxblox_ros/esdf_server.h>

#include <random>
#include <algorithm>
#include <memory>

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
        ros::Publisher evaluation_vis_pub_;
        ros::ServiceServer run_srv_;

        // members
        voxblox::EsdfServer voxblox_server_;
        TrajectoryGenerator* trajectory_generator_;
        TrajectoryEvaluator* trajectory_evaluator_;

        // variables
        std::shared_ptr<TrajectorySegment> current_segment_;        // root node to full tree
        bool running_;
        Eigen::Vector3d target_position_;
        int vis_num_previous_trajectories_;
        ros::Time info_timing_;

        // params
        double p_replan_threshold_;
        std::string p_TG_name_;
        std::string p_TE_name_;
        bool p_coll_optimistic_;
        double p_coll_radius_;
        bool p_verbose_;

        // methods
        void setParamsFromRos();
        void initializePlanning();
        void requestNextTrajectory();
        void expandTrajectories();
        void requestMovement(const TrajectorySegment &req);
        void publishTrajectoryVisualization(const std::vector<TrajectorySegment*> &trajectories);
        void publishEvalVisualization(const TrajectorySegment &trajectory);
    };

    PlannerNode::PlannerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
            : nh_(nh),
              nh_private_(nh_private),
              voxblox_server_(nh_, nh_private_) {

        // initial values and params
        setParamsFromRos();
        running_ = false;
        vis_num_previous_trajectories_ = 0;

        // Subscribers and publishers
        target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory_visualization", 100);
        evaluation_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("evaluation_visualization", 300);
        odom_sub_ = nh_.subscribe("odometry", 1, &PlannerNode::odomCallback, this);

        // Setup trajectory generator
        if (p_TG_name_ == "RandomLinear") {
            trajectory_generator_ = new TGRandomLinear(&voxblox_server_, p_coll_optimistic_, p_coll_radius_);
        } else if (p_TG_name_ == "MavTrajectoryGeneration") {
            trajectory_generator_ = new TGMavTrajectoryGeneration(&voxblox_server_, p_coll_optimistic_, p_coll_radius_);
        } else if (p_TG_name_ == "Uniform") {
            trajectory_generator_ = new TGUniform(&voxblox_server_, p_coll_optimistic_, p_coll_radius_);
        } else{
            ROS_ERROR("Unknown trajectory generator '%s'.", p_TG_name_.c_str());
        }
        trajectory_generator_->setParamsFromRos(nh_private_);

        // Setup trajectory evaluator
        if (p_TE_name_ == "Naive") {
            trajectory_evaluator_ = new TENaive(&voxblox_server_);
        } else {
            ROS_ERROR("Unknown trajectory evaluator '%s'.", p_TE_name_.c_str());
        }
        trajectory_evaluator_->setParamsFromRos(nh_private_);

        // setup voxblox collision checking and wait for first map
        ros::topic::waitForMessage<voxblox_msgs::Layer>("esdf_map_in", nh_private_);
        voxblox_server_.setTraversabilityRadius(p_coll_radius_);
        voxblox_server_.publishTraversable();

        // Finish
        run_srv_ = nh_private_.advertiseService("toggle_running", &PlannerNode::runService, this);
    }

    void PlannerNode::setParamsFromRos() {
        nh_private_.param("verbose", p_verbose_, true);
        nh_private_.param("trajectory_generator", p_TG_name_, std::string("RandomLinear"));
        nh_private_.param("trajectory_evaluator", p_TE_name_, std::string("Naive"));
        nh_private_.param("collision_optimistic", p_coll_optimistic_, false);
        nh_private_.param("collision_radius", p_coll_radius_, 0.35);
        nh_private_.param("replan_threshold", p_replan_threshold_, 0.05);
    }

    void PlannerNode::odomCallback(const nav_msgs::Odometry &msg) {
        // This is the main loop, high odom message frequency is expected to continuously run
        if (!running_) { return; }
        geometry_msgs::Point pos = msg.pose.pose.position;
        Eigen::Vector3d position(pos.x, pos.y, pos.z);
        double distance = (target_position_ - position).squaredNorm();
        if (distance < p_replan_threshold_ && !current_segment_->children.empty()) {
            // After finishing current segment execute next one
            requestNextTrajectory();
        } else {
            // Continuosly expand the trajectory space
            expandTrajectories();
        }
    }

    void PlannerNode::initializePlanning() {
        // Initialize and start planning (Currently assumes MAV starts at {0,0,0}!)
        target_position_ = Eigen::Vector3d(0, 0, 0);

        // Setup initial trajectory Segment
        current_segment_ = std::make_shared<TrajectorySegment>();
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = target_position_;
        trajectory_point.setFromYaw(0);
        current_segment_->trajectory.push_back(trajectory_point);

        info_timing_ = ros::Time::now();
    }

    void PlannerNode::requestNextTrajectory() {
        // Select best next trajectory (Note: current_segment->children must not be empty!)
        int next_segment = trajectory_evaluator_ -> selectNextBest(*current_segment_);

        // Visualize trajectories
        std::vector<TrajectorySegment*> trajectories_to_vis;
        current_segment_->getTree(trajectories_to_vis);
        publishTrajectoryVisualization(trajectories_to_vis);
        if (p_verbose_){
            ROS_INFO("Replanning! (Evaluated %i segments in %.3f seconds)", (int)trajectories_to_vis.size()-1,
                    (ros::Time::now()-info_timing_).toSec());
            info_timing_ = ros::Time::now();
        }

        // Reset tree (Note: could also keep subtree and reevaluate but doesnt matter for this first case)
        current_segment_ = current_segment_->children[next_segment];
        current_segment_->parent = nullptr;
        current_segment_->children.clear();

        // Move
        requestMovement(*current_segment_);

        // Force Esdf update, so next trajectories can be evaluated
        voxblox_server_.updateEsdf();

        // Visualization of new info
        voxblox_server_.publishTraversable();
        publishEvalVisualization(*current_segment_);
    }

    void PlannerNode::expandTrajectories() {
        // Select and expand a segment
        TrajectorySegment* expansion_target = trajectory_generator_->selectSegment(*current_segment_);
        int previous_children = expansion_target->children.size();
        trajectory_generator_->expandSegment(*expansion_target);

        // Evaluate newly added segments
        for (int i=previous_children; i < expansion_target->children.size(); ++i) {
            trajectory_evaluator_->computeGain(*(expansion_target->children[i]));
            trajectory_evaluator_->computeCost(*(expansion_target->children[i]));
            trajectory_evaluator_->computeValue(*(expansion_target->children[i]));
        }
    }

    void PlannerNode::requestMovement(const TrajectorySegment &req) {
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

    void PlannerNode::publishTrajectoryVisualization(const std::vector<TrajectorySegment*> &trajectories) {
        // Display all trajectories in the input and erase previous ones
        double max_value = (*std::max_element(trajectories.begin(), trajectories.end(), TrajectorySegment::comparePtr))->value;
        double min_value = (*std::min_element(trajectories.begin(), trajectories.end(), TrajectorySegment::comparePtr))->value;
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
            msg.action = visualization_msgs::Marker::ADD;

            // Color according to relative cost
            double frac = (trajectories[i]->value - min_value) / (max_value - min_value);
            msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
            msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
            msg.color.b = 0.0;
            msg.color.a = 0.4;

            // points
            for (int j = 0; j < trajectories[i]->trajectory.size(); ++j) {
                geometry_msgs::Point point;
                point.x = trajectories[i]->trajectory[j].position_W[0];
                point.y = trajectories[i]->trajectory[j].position_W[1];
                point.z = trajectories[i]->trajectory[j].position_W[2];
                msg.points.push_back(point);
            }
            trajectory_vis_pub_.publish(msg);
        }
        for (int i = trajectories.size(); i < vis_num_previous_trajectories_; ++i) {
            // Setup marker message
            visualization_msgs::Marker msg;
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            msg.type = visualization_msgs::Marker::POINTS;
            msg.id = i;
            msg.ns = "candidate_trajectories";
            msg.action = visualization_msgs::Marker::DELETE;
            trajectory_vis_pub_.publish(msg);
        }
        vis_num_previous_trajectories_ = trajectories.size();
    }

    void PlannerNode::publishEvalVisualization(const TrajectorySegment &trajectory) {
        // Setup marker message
        visualization_msgs::Marker msg;
        msg.header.frame_id = "/world";
        msg.header.stamp = ros::Time::now();
        msg.pose.orientation.w = 1.0;
        msg.type = visualization_msgs::Marker::CUBE_LIST;
        msg.ns = "Evaluation";
        voxblox::FloatingPoint voxel_size = voxblox_server_.getEsdfMapPtr()->voxel_size();
        voxblox::FloatingPoint block_size = voxblox_server_.getEsdfMapPtr()->block_size();
        msg.scale.x = (double) voxel_size;
        msg.scale.y = (double) voxel_size;
        msg.scale.z = (double) voxel_size;
        msg.color.r = 1.0;
        msg.color.g = 0.8;
        msg.color.b = 0.0;
        msg.color.a = 0.3;

        // points
        int voxels_per_side = (int) std::round(block_size / voxel_size);
        voxblox::BlockIndex block_id;
        voxblox::VoxelIndex voxel_id;
        for (int i = 0; i < trajectory.info.size(); ++i) {
            voxblox::getBlockAndVoxelIndexFromGlobalVoxelIndex(trajectory.info[i], voxels_per_side, &block_id,
                                                               &voxel_id);
            voxblox::Point voxbloxpoint = voxblox::getOriginPointFromGridIndex(block_id, block_size) +
                                          voxblox::getCenterPointFromGridIndex(voxel_id, voxel_size);
            geometry_msgs::Point point;
            point.x = (double) voxbloxpoint.x();
            point.y = (double) voxbloxpoint.y();
            point.z = (double) voxbloxpoint.z();
            msg.points.push_back(point);
        }
        evaluation_vis_pub_.publish(msg);
    }

    bool PlannerNode::runService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        res.success = true;
        if (req.data) {
            ROS_INFO("Started planning.");
            initializePlanning();
            running_ = true;
        } else {
            ROS_INFO("Stopped planning.");
            running_ = false;
        }
        return true;
    }

} // namespace mav_active_3d_planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "active_3d_planner");

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::PlannerNode planner_node(nh, nh_private);

    ROS_INFO("Initialized active_3d_planner_node.");
    ros::spin();
}