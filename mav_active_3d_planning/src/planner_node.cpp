#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/trajectory_evaluator.h"
#include "mav_active_3d_planning/back_tracker.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <ros/ros.h>
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
#include <string>
#include <cmath>
#include <ctime>
#include <iostream>
#include <sstream>
#include <fstream>


namespace mav_active_3d_planning {

    class PlannerNode {
    public:
        PlannerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
        virtual ~PlannerNode() {}

        // ros callbacks
        void odomCallback(const nav_msgs::Odometry &msg);
        bool runSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
        bool cpuSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

    protected:
        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber odom_sub_;
        ros::Publisher target_pub_;
        ros::Publisher trajectory_vis_pub_;
        ros::ServiceServer run_srv_;
        ros::ServiceServer get_cpu_time_srv_;

        // members
        std::shared_ptr<voxblox::EsdfServer> voxblox_server_;
        std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
        std::unique_ptr<TrajectoryEvaluator> trajectory_evaluator_;
        std::unique_ptr<BackTracker> back_tracker_;

        // variables
        std::unique_ptr<TrajectorySegment> current_segment_;        // root node of full trajectory tree
        bool running_;                      // whether to run the main loop
        Eigen::Vector3d target_position_;   // current movement goal
        double target_yaw_;
        int vis_num_previous_trajectories_; // Counter for visualization function
        int vis_completed_count_;
        int new_segments_;                  // keep track of min/max tries and segments
        int new_segment_tries_;

        // Info+performance bookkeeping
        ros::Time info_timing_;             // Rostime for verbose and perf
        int info_count_;                    // num trajectories counter for verbose
        std::ofstream perf_log_file_;       // performance file
        std::vector<double> perf_log_data_; // select, expand, gain, cost, value [cpu seconds]
        std::clock_t perf_cpu_timer_;       // total time counter
        std::clock_t cpu_srv_timer_;        // To get CPU usage for service

        // params
        double p_replan_pos_threshold_;     // m
        double p_replan_yaw_threshold_;     // rad
        bool p_verbose_;
        bool p_visualize_candidates_;
        bool p_log_performance_;            // Whether to write a performance log file
        int p_max_new_segments_;
        int p_min_new_segments_;
        int p_min_new_tries_;
        int p_max_new_tries_;
        // ideas: take images only on points,

        // methods
        void initializePlanning();
        void requestNextTrajectory();
        void expandTrajectories();
        void requestMovement(const TrajectorySegment &req);

        // visualization
        void publishTrajectoryVisualization(const std::vector<TrajectorySegment*> &trajectories);
        void publishCompletedTrajectoryVisualization(const TrajectorySegment &trajectories);
        void publishEvalVisualization(const TrajectorySegment &trajectory);
    };

    PlannerNode::PlannerNode(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
            : nh_(nh),
              nh_private_(nh_private),
              running_(false),
              vis_num_previous_trajectories_(0),
              vis_completed_count_(0) {

        // Initialize a random seed once
        srand(time(NULL));

        // Params
        bool verbose_modules;
        bool build_modules_on_init;

        // When to start next trajectory, use 0 for not relevant
        nh_private_.param("replan_pos_threshold", p_replan_pos_threshold_, 0.1);
        nh_private_.param("replan_yaw_threshold", p_replan_yaw_threshold_, 0.2);

        // Logging and printing
        nh_private_.param("verbose", p_verbose_, true);
        nh_private_.param("verbose_modules", verbose_modules, false);
        nh_private_.param("build_modules_on_init", build_modules_on_init, false);
        nh_private_.param("visualize_candidates", p_visualize_candidates_, true);
        nh_private_.param("log_performance", p_log_performance_, false);

        // Sampling constraints
        nh_private_.param("max_new_segments", p_max_new_segments_, 0);  // set 0 for infinite
        nh_private_.param("min_new_segments", p_min_new_segments_, 0);
        nh_private_.param("max_new_tries", p_max_new_tries_, 0);  // set 0 for infinite
        nh_private_.param("min_new_tries", p_min_new_tries_, 0);

        // Wait for voxblox to setup and receive first map
        ros::topic::waitForMessage<voxblox_msgs::Layer>("esdf_map_in", nh_private_);

        // Setup members
        std::string ns = ros::this_node::getName();
        voxblox_server_ = std::shared_ptr<voxblox::EsdfServer>( new voxblox::EsdfServer(nh_, nh_private_));
        trajectory_generator_ = ModuleFactory::Instance()->createTrajectoryGenerator(
                ns + "/trajectory_generator", voxblox_server_, verbose_modules);
        trajectory_evaluator_ = ModuleFactory::Instance()->createTrajectoryEvaluator(
                ns + "/trajectory_evaluator", voxblox_server_, verbose_modules);
        back_tracker_ = ModuleFactory::Instance()->createBackTracker(ns + "/back_tracker", verbose_modules);

        // Force lazy initialization of modules (call every function once)
        if (build_modules_on_init) {
            // Empty set of arguments required to run everything
            TrajectorySegment temp_segment;
            TrajectorySegment* temp_pointer;
            std::vector<TrajectorySegment*> temp_vector;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = Eigen::Vector3d(0, 0, 0);
            trajectory_point.setFromYaw(0);
            temp_segment.trajectory.push_back(trajectory_point);
            temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
//            auto resetTemps = [](TrajectorySegment* temp_segment, TrajectorySegment** temp_pointer,
//                    std::vector<TrajectorySegment*> *temp_vector) {
//                // Empty set of arguments required to run everything
//                *temp_segment = TrajectorySegment();
//                mav_msgs::EigenTrajectoryPoint trajectory_point;
//                trajectory_point.position_W = Eigen::Vector3d(0, 0, 0);
//                trajectory_point.setFromYaw(0);
//                temp_segment->trajectory.push_back(trajectory_point);
//                *temp_pointer = temp_segment->spawnChild();
//                (*temp_pointer)->trajectory.push_back(trajectory_point);
//                temp_vector->clear();
//            };
//            resetTemps(&temp_segment, &temp_pointer, &temp_vector);
            trajectory_generator_->selectSegment(&temp_pointer, &temp_segment);
            trajectory_generator_->expandSegment(temp_pointer, &temp_vector);
            trajectory_generator_->updateSegments(&temp_segment);

            temp_segment = TrajectorySegment();
            temp_segment.trajectory.push_back(trajectory_point);
            trajectory_evaluator_->computeGain(&temp_segment);
            trajectory_evaluator_->computeCost(&temp_segment);
            trajectory_evaluator_->computeValue(&temp_segment);
            trajectory_evaluator_->updateSegments(&temp_segment);

            temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
            trajectory_evaluator_->selectNextBest(temp_segment);
        }

        // Subscribers and publishers
        target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        trajectory_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("trajectory_visualization", 1000);
        odom_sub_ = nh_.subscribe("odometry", 2, &PlannerNode::odomCallback, this);

        // Finish
        run_srv_ = nh_private_.advertiseService("toggle_running", &PlannerNode::runSrvCallback, this);
        get_cpu_time_srv_ = nh_private_.advertiseService("get_cpu_time", &PlannerNode::cpuSrvCallback, this);
    }

    void PlannerNode::odomCallback(const nav_msgs::Odometry &msg) {
        // This is the main loop, high odom message frequency is expected to continuously run
        if (!running_) { return; }
        Eigen::Vector3d position(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
        double yaw = tf::getYaw(msg.pose.pose.orientation);

        // check goal pos reached (if tol is set)
        if (p_replan_pos_threshold_ <= 0 || (target_position_ - position).norm() < p_replan_pos_threshold_) {
            // check goal yaw reached (if tol is set)
            if (p_replan_yaw_threshold_ <= 0 || defaults::angleDifference(target_yaw_, yaw) < p_replan_yaw_threshold_) {
                // check minimum tries reached
                if (new_segment_tries_ >= p_min_new_tries_) {
                    if (new_segments_ >= p_min_new_segments_ ||
                        (new_segment_tries_ >= p_max_new_tries_ && p_max_new_tries_ > 0)) {
                        // After finishing the current segment execute next one
                        requestNextTrajectory();
                        return;
                    }
                }
            }
        }

        // Otherwise continuosly expand the trajectory space
        expandTrajectories();
    }

    void PlannerNode::initializePlanning() {
        // Initialize and start planning (Currently assumes MAV starts at {0,0,0} with 0 yaw!)
        target_position_ = Eigen::Vector3d(0, 0, 0);
        target_yaw_ = 0.0;

        // Setup initial trajectory Segment
        current_segment_ = std::unique_ptr<TrajectorySegment>(new TrajectorySegment());
        mav_msgs::EigenTrajectoryPoint trajectory_point;
        trajectory_point.position_W = target_position_;
        trajectory_point.setFromYaw(target_yaw_);
        current_segment_->trajectory.push_back(trajectory_point);

        // Setup performance log
        if (p_log_performance_) {
            std::string log_dir("");
            if (!nh_private_.getParam("performance_log_dir", log_dir)) {
                // Wait for this param until planning start so it can be set after constructor (e.g. from other nodes)
                ROS_WARN("No directory for performance log set, performance log turned off.");
                p_log_performance_ = false;
            } else {
                perf_log_file_.open(log_dir + "/performance_log.csv");
                perf_log_data_ = std::vector<double>(5, 0.0); //select, expand, gain, cost, value
                perf_cpu_timer_ = std::clock();
                perf_log_file_ << "RosTime,NTrajectories,NTrajAfterUpdate,Select,Expand,Gain,Cost,Value,NextBest,"
                                  "UpdateTG,UpdateTE,Visualization,Total" << std::endl;
            }
        }

        // Setup counters
        cpu_srv_timer_ = std::clock();
        info_timing_ = ros::Time::now();
        info_count_ = 0;
        new_segments_ = 0;
        new_segment_tries_ = 0;
    }

    void PlannerNode::requestNextTrajectory() {
        if (current_segment_->children.empty()) {
            // No trajectories available: call the backtracker
            back_tracker_->trackBack(current_segment_.get());
            return;
        }

        // Performance tracking
        double perf_rostime;
        double perf_vis = 0.0;
        double perf_next;
        double perf_uptg;
        double perf_upte;
        std::clock_t timer;
        if (p_log_performance_) {
            timer = std::clock();
        }

        // Visualize candidates
        std::vector < TrajectorySegment * > trajectories_to_vis;
        current_segment_->getTree(&trajectories_to_vis);
        if (p_visualize_candidates_){
            publishTrajectoryVisualization(trajectories_to_vis);
        }
        int num_trajectories = trajectories_to_vis.size();
        if (p_verbose_ || p_log_performance_) {
            perf_rostime = (ros::Time::now() - info_timing_).toSec();
            info_timing_ = ros::Time::now();
            if (p_verbose_) {
                ROS_INFO("Replanning! Found %i new segments (%i total) in %.3f seconds.",
                         num_trajectories - info_count_, num_trajectories, perf_rostime);
            }
        }
        if (p_log_performance_) {
            perf_vis += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Select best next trajectory and update root
        int next_segment = trajectory_evaluator_->selectNextBest(*current_segment_);
        current_segment_ = std::move(current_segment_->children[next_segment]);
        current_segment_->parent = nullptr;
        current_segment_->gain = 0.0;
        current_segment_->cost = 0.0;
        current_segment_->value = 0.0;
        if (p_log_performance_) {
            perf_next = (double) (std::clock() - timer) / CLOCKS_PER_SEC;
        }

        // Move
        requestMovement(*current_segment_);
        back_tracker_->segmentIsExecuted(*current_segment_.get());

        // Force Esdf update, so next trajectories can be evaluated
        voxblox_server_->updateEsdf();

        // Visualize
        if (p_log_performance_) {
            timer = std::clock();
        }
        if (p_visualize_candidates_) {
            voxblox_server_->publishTraversable();
            publishEvalVisualization(*current_segment_);
        }
        publishCompletedTrajectoryVisualization(*current_segment_);
        if (p_log_performance_) {
            perf_vis += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Update tree
        trajectory_generator_->updateSegments(current_segment_.get());
        if (p_log_performance_) {
            perf_uptg = (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        trajectory_evaluator_->updateSegments(current_segment_.get());
        if (p_log_performance_) {
            perf_upte = (double) (std::clock() - timer) / CLOCKS_PER_SEC;
        }

        // Performance log
        if (p_verbose_ || p_log_performance_) {
            std::vector < TrajectorySegment * > trajectory_count;
            current_segment_->getTree(&trajectory_count);
            info_count_ = trajectory_count.size();

            if (p_log_performance_) {
                perf_log_file_ << perf_rostime << "," << num_trajectories << "," << info_count_ << "," <<
                               perf_log_data_[0] << "," << perf_log_data_[1] << "," << perf_log_data_[2] << "," <<
                               perf_log_data_[3] << "," << perf_log_data_[4] << "," << perf_next << "," <<
                               perf_uptg << "," << perf_upte << "," << perf_vis << "," <<
                               (double) (std::clock() - perf_cpu_timer_) / CLOCKS_PER_SEC << std::endl;
                perf_log_data_ = std::vector<double>(5, 0.0);
                perf_cpu_timer_ = std::clock();
            }
        }

        // Update tracking values
        new_segment_tries_ = 0;
        new_segments_ = 0;
    }

    void PlannerNode::expandTrajectories() {
        // Check max number of tries already
        if (p_max_new_segments_ > 0 && new_segments_ >= p_max_new_segments_) {
            return;
        }

        // Select expansion target
        std::clock_t timer;
        if (p_log_performance_) {
            timer = std::clock();
        }
        TrajectorySegment *expansion_target;
        trajectory_generator_->selectSegment(&expansion_target, current_segment_.get());
        if (p_log_performance_) {
            perf_log_data_[0] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Expand the target
        std::vector<TrajectorySegment*> created_segments;
        bool success = trajectory_generator_->expandSegment(expansion_target, &created_segments);
        if (p_log_performance_) {
            perf_log_data_[1] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }
        new_segment_tries_++;
        if (success) { new_segments_++; }

        // Evaluate newly added segments: Gain
        for (int i = 0; i < created_segments.size(); ++i) {
            trajectory_evaluator_->computeGain(created_segments[i]);
        }
        if (p_log_performance_) {
            perf_log_data_[2] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Costs
        for (int i = 0; i < created_segments.size(); ++i) {
            trajectory_evaluator_->computeCost(created_segments[i]);
        }
        if (p_log_performance_) {
            perf_log_data_[3] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Final value
        for (int i = 0; i < created_segments.size(); ++i) {
            trajectory_evaluator_->computeValue(created_segments[i]);
        }
        if (p_log_performance_) {
            perf_log_data_[4] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
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
        target_yaw_ = req.trajectory.back().getYaw();
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

            // Text
            visualization_msgs::Marker msg2;
            msg2.header.frame_id = "/world";
            msg2.header.stamp = ros::Time::now();
            msg2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            msg2.id = i;
            msg2.ns = "candidate_text";
            msg2.scale.x = 0.2;
            msg2.scale.y = 0.2;
            msg2.scale.z = 0.2;

            msg2.color.r = 0.0f;
            msg2.color.g = 0.0f;
            msg2.color.b = 0.0f;
            msg2.color.a = 1.0;
            msg2.pose.position.x = trajectories[i]->trajectory.back().position_W[0];
            msg2.pose.position.y = trajectories[i]->trajectory.back().position_W[1];
            msg2.pose.position.z = trajectories[i]->trajectory.back().position_W[2];
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << trajectories[i]->gain << "/" << std::fixed <<
                   std::setprecision(2) << trajectories[i]->cost << "/" << std::fixed << std::setprecision(2) <<
                   trajectories[i]->value;
            msg2.text = stream.str();
            msg2.action = visualization_msgs::Marker::ADD;
            trajectory_vis_pub_.publish(msg2);
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

            // Text
            visualization_msgs::Marker msg2;
            msg2.header.frame_id = "/world";
            msg2.header.stamp = ros::Time::now();
            msg2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            msg2.id = i;
            msg2.ns = "candidate_text";
            msg2.action = visualization_msgs::Marker::DELETE;
            trajectory_vis_pub_.publish(msg2);

        }
        vis_num_previous_trajectories_ = trajectories.size();
    }

    void PlannerNode::publishCompletedTrajectoryVisualization(const TrajectorySegment &trajectories) {
        // Continuously increment the already traveled path
        visualization_msgs::Marker msg;
        msg.header.frame_id = "/world";
        msg.header.stamp = ros::Time::now();
        msg.pose.orientation.w = 1.0;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.ns = "completed_trajectory";
        msg.id = vis_completed_count_;
        msg.scale.x = 0.03;
        msg.scale.y = 0.03;
        msg.action = visualization_msgs::Marker::ADD;
        msg.color.r = 0.5;
        msg.color.g = 0.5;
        msg.color.b = 0.5;
        msg.color.a = 0.8;

        // points
        for (int i = 0; i < trajectories.trajectory.size(); ++i) {
            geometry_msgs::Point point;
            point.x = trajectories.trajectory[i].position_W[0];
            point.y = trajectories.trajectory[i].position_W[1];
            point.z = trajectories.trajectory[i].position_W[2];
            msg.points.push_back(point);
        }
        trajectory_vis_pub_.publish(msg);
        vis_completed_count_++;
    }

    void PlannerNode::publishEvalVisualization(const TrajectorySegment &trajectory) {
        // Visualize the gain of the current segment
        visualization_msgs::Marker msg;
        trajectory_evaluator_->visualizeTrajectoryValue(&msg, trajectory);
        msg.ns = "evaluation";
        msg.header.stamp = ros::Time::now();
        trajectory_vis_pub_.publish(msg);
    }

    bool PlannerNode::runSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
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

    bool PlannerNode::cpuSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        double time = (double)(std::clock() - cpu_srv_timer_) / CLOCKS_PER_SEC;
        cpu_srv_timer_ = std::clock();

        // Just return cpu time as the service message
        res.message = std::to_string(time).c_str();
        res.success = true;
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