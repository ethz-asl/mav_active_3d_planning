#define _USE_MATH_DEFINES
#define _POSIX_SOURCE

#include "mav_active_3d_planning/planner_node.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>

#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>


namespace mav_active_3d_planning {

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
        nh_private_.param("replan_yaw_threshold", p_replan_yaw_threshold_, 0.1);

        // Logging and printing
        nh_private_.param("verbose", p_verbose_, true);
        nh_private_.param("verbose_modules", verbose_modules, false);
        nh_private_.param("build_modules_on_init", build_modules_on_init, false);
        nh_private_.param("visualize", p_visualize_, true);
        nh_private_.param("publish_traversable", p_publish_traversable_, false);    // As in voxblox
        nh_private_.param("log_performance", p_log_performance_, false);
        if (p_log_performance_) {
            perf_log_data_ = std::vector<double>(6, 0.0);
        }

        // Sampling constraints
        nh_private_.param("max_new_segments", p_max_new_segments_, 0);  // set 0 for infinite
        nh_private_.param("min_new_segments", p_min_new_segments_, 0);
        nh_private_.param("max_new_tries", p_max_new_tries_, 0);  // set 0 for infinite
        nh_private_.param("min_new_tries", p_min_new_tries_, 0);
        nh_private_.param("min_new_value", p_min_new_value_, 0.0);
        nh_private_.param("expand_batch", p_expand_batch_, 1);
        p_expand_batch_ = std::max(p_expand_batch_, 1);

        // Wait for voxblox to setup and receive first map
        ros::topic::waitForMessage<voxblox_msgs::Layer>("esdf_map_in", nh_private_);

        // Setup members
        ModuleFactory::Instance()->registerLinkableModule("PlannerNode", this);
        std::string ns = ros::this_node::getName();
        voxblox_server_.reset(new voxblox::EsdfServer(nh_, nh_private_));
        trajectory_generator_ = ModuleFactory::Instance()->createModule<TrajectoryGenerator>(
                ns + "/trajectory_generator", verbose_modules);
        trajectory_evaluator_ = ModuleFactory::Instance()->createModule<TrajectoryEvaluator>(
                ns + "/trajectory_evaluator", verbose_modules);
        back_tracker_ = ModuleFactory::Instance()->createModule<BackTracker>(ns + "/back_tracker", verbose_modules);

        // Force lazy initialization of modules (call every function once)
        if (build_modules_on_init) {
            // Empty set of arguments required to run everything
            TrajectorySegment temp_segment;
            TrajectorySegment *temp_pointer;
            std::vector < TrajectorySegment * > temp_vector;
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_point.position_W = Eigen::Vector3d(0, 0, 0);
            trajectory_point.setFromYaw(0);
            temp_segment.trajectory.push_back(trajectory_point);
            temp_segment.spawnChild()->trajectory.push_back(trajectory_point);
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
            trajectory_evaluator_->selectNextBest(&temp_segment);
        }

        // Subscribers and publishers
        target_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        trajectory_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_visualization", 100);
        odom_sub_ = nh_.subscribe("odometry", 1, &PlannerNode::odomCallback, this);
        get_cpu_time_srv_ = nh_private_.advertiseService("get_cpu_time", &PlannerNode::cpuSrvCallback, this);

        // Finish
        run_srv_ = nh_private_.advertiseService("toggle_running", &PlannerNode::runSrvCallback, this);
    }

    void PlannerNode::initializePlanning() {
        // Setup initial trajectory Segment
        target_position_ = current_position_;
        target_yaw_ = tf::getYaw(tf::Quaternion(current_orientation_.x(), current_orientation_.y(), current_orientation_.z(),
                current_orientation_.w()));   // tf yaw gets better results than eigen-euler conversion here
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
                logfile_ = log_dir + "/performance_log.csv";
                perf_log_file_.open(logfile_.c_str());
                perf_log_data_ = std::vector<double>(6, 0.0); //select, expand, gain, cost, value, mainLoop
                perf_cpu_timer_ = std::clock();
                perf_log_file_ << "RosTime,NTrajectories,NTrajAfterUpdate,Select,Expand,Gain,Cost,Value,NextBest,"
                                  "UpdateTG,UpdateTE,Visualization,RosCallbacks,Total" << std::endl;
            }
        }

        // Setup counters
        cpu_srv_timer_ = std::clock();
        info_timing_ = ros::Time::now();
        info_count_ = 1;
        new_segments_ = 0;
        new_segment_tries_ = 0;
        vis_num_previous_evaluations_ = 0;
        vis_num_previous_trajectories_ = 0;
        min_new_value_reached_ = p_min_new_value_ == 0.0;
        target_reached_ = true;
    }

    void PlannerNode::planningLoop() {
        // This is the main loop, spinning is managed explicitely for efficiency
        std::clock_t timer;
        while (ros::ok()) {
            if (running_) {
                loopIteration();
            }
            if (p_log_performance_) {
                timer = std::clock();
            }
            ros::spinOnce();
            if (p_log_performance_) {
                perf_log_data_[5] += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            }
        }
    }

    void PlannerNode::loopIteration() {
        // Continuosly expand the trajectory space
        for (int i = 0; i < p_expand_batch_; ++i) {
            expandTrajectories();
        }

        if (!min_new_value_reached_) {
            //  recursively check whether the minimum value is reached
            min_new_value_reached_ = checkMinNewValue(current_segment_);
        }

        // After finishing the current segment, execute the next one
        if (target_reached_) {
            if (new_segment_tries_ >= p_max_new_tries_ && p_max_new_tries_ > 0) {
                // Maximum tries reached: force next segment
                requestNextTrajectory();
            } else {
                if (new_segment_tries_ < p_min_new_tries_) {
                    return;     // check minimum tries reached
                }
                if (new_segments_ < p_min_new_segments_) {
                    return;     // check minimum successful expansions reached
                }
                if (!min_new_value_reached_) {
                    return;      // check minimum value reached
                }
                // All requirements met
                requestNextTrajectory();
            }
        }
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
        trajectories_to_vis.erase(trajectories_to_vis.begin()); // remove current segment (root)
        if (p_visualize_) {
            publishTrajectoryVisualization(trajectories_to_vis);
        }
        int num_trajectories = trajectories_to_vis.size();
        if (p_verbose_ || p_log_performance_) {
            perf_rostime = (ros::Time::now() - info_timing_).toSec();
            info_timing_ = ros::Time::now();
            if (p_verbose_) {
                ROS_INFO("Replanning! Found %i new segments (%i total) in %.2f seconds.",
                         num_trajectories - info_count_ + 1, num_trajectories, perf_rostime);
            }
        }
        if (p_log_performance_) {
            perf_vis += (double) (std::clock() - timer) / CLOCKS_PER_SEC;
            timer = std::clock();
        }

        // Select best next trajectory and update root
        int next_segment = trajectory_evaluator_->selectNextBest(current_segment_.get());
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
        if (p_visualize_) {
            if (p_publish_traversable_) {
                voxblox_server_->publishTraversable();
            }
            publishEvalVisualization(*current_segment_);
            publishCompletedTrajectoryVisualization(*current_segment_);
        }
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
                               perf_uptg << "," << perf_upte << "," << perf_vis << "," << perf_log_data_[5] << "," <<
                               (double) (std::clock() - perf_cpu_timer_) / CLOCKS_PER_SEC << "\n";
                perf_log_data_ = std::vector<double>(6, 0.0);
                perf_cpu_timer_ = std::clock();
            }
        }

        // Update tracking values
        new_segment_tries_ = 0;
        new_segments_ = 0;
        min_new_value_reached_ = p_min_new_value_ == 0.0;
        target_reached_ = false;
    }

    void PlannerNode::expandTrajectories() {
        // Check max number of tries already and min value found
        if (p_max_new_segments_ > 0 && new_segments_ >= p_max_new_segments_ && min_new_value_reached_) {
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
        std::vector < TrajectorySegment * > created_segments;
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

    void PlannerNode::odomCallback(const nav_msgs::Odometry &msg) {
        // Track the current pose
        current_position_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                            msg.pose.pose.position.z);
        current_orientation_ = Eigen::Quaterniond(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        if (running_ && !target_reached_) {
            // check goal pos reached (if tol is set)
            if (p_replan_pos_threshold_ <= 0 ||
                (target_position_ - current_position_).norm() < p_replan_pos_threshold_) {
                // check goal yaw reached (if tol is set)
                double yaw = tf::getYaw(msg.pose.pose.orientation);
                if (p_replan_yaw_threshold_ <= 0 ||
                    defaults::angleDifference(target_yaw_, yaw) < p_replan_yaw_threshold_) {
                    target_reached_ = true;
                }
            }
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

    void PlannerNode::publishTrajectoryVisualization(const std::vector<TrajectorySegment *> &trajectories) {
        // Display all trajectories in the input and erase previous ones
        double max_value = (*std::max_element(trajectories.begin(), trajectories.end(),
                                              TrajectorySegment::comparePtr))->value;
        double min_value = (*std::min_element(trajectories.begin(), trajectories.end(),
                                              TrajectorySegment::comparePtr))->value;

        visualization_msgs::MarkerArray array_msg;
        visualization_msgs::Marker msg;
        for (int i = 0; i < trajectories.size(); ++i) {
            // Setup marker message
            msg = visualization_msgs::Marker();
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            msg.pose.orientation.w = 1.0;
            msg.type = visualization_msgs::Marker::POINTS;
            msg.id = i;
            msg.ns = "candidate_trajectories";
            msg.scale.x = 0.04;
            msg.scale.y = 0.04;
            msg.action = visualization_msgs::Marker::ADD;

            // Color according to relative value (blue when indifferent)
            if (max_value != min_value) {
                double frac = (trajectories[i]->value - min_value) / (max_value - min_value);
                msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
                msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
                msg.color.b = 0.0;
            } else {
                msg.color.r = 0.3;
                msg.color.g = 0.3;
                msg.color.b = 1.0;
            }

            msg.color.a = 0.4;

            // points
            for (int j = 0; j < trajectories[i]->trajectory.size(); ++j) {
                geometry_msgs::Point point;
                point.x = trajectories[i]->trajectory[j].position_W[0];
                point.y = trajectories[i]->trajectory[j].position_W[1];
                point.z = trajectories[i]->trajectory[j].position_W[2];
                msg.points.push_back(point);
            }
            array_msg.markers.push_back(msg);

            // Text
            msg = visualization_msgs::Marker();
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            msg.id = i;
            msg.ns = "candidate_text";
            msg.scale.x = 0.2;
            msg.scale.y = 0.2;
            msg.scale.z = 0.2;

            msg.color.r = 0.0f;
            msg.color.g = 0.0f;
            msg.color.b = 0.0f;
            msg.color.a = 1.0;
            msg.pose.position.x = trajectories[i]->trajectory.back().position_W[0];
            msg.pose.position.y = trajectories[i]->trajectory.back().position_W[1];
            msg.pose.position.z = trajectories[i]->trajectory.back().position_W[2];
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << trajectories[i]->gain << "/" << std::fixed <<
                   std::setprecision(2) << trajectories[i]->cost << "/" << std::fixed << std::setprecision(2) <<
                   trajectories[i]->value;
            msg.text = stream.str();
            msg.action = visualization_msgs::Marker::ADD;
            array_msg.markers.push_back(msg);
        }
        for (int i = trajectories.size(); i < vis_num_previous_trajectories_; ++i) {
            // Setup marker message
            msg = visualization_msgs::Marker();
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            msg.type = visualization_msgs::Marker::POINTS;
            msg.id = i;
            msg.ns = "candidate_trajectories";
            msg.action = visualization_msgs::Marker::DELETE;
            array_msg.markers.push_back(msg);

            // Text
            msg = visualization_msgs::Marker();
            msg.header.frame_id = "/world";
            msg.header.stamp = ros::Time::now();
            msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            msg.id = i;
            msg.ns = "candidate_text";
            msg.action = visualization_msgs::Marker::DELETE;
            array_msg.markers.push_back(msg);
        }
        trajectory_vis_pub_.publish(array_msg);
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
        visualization_msgs::MarkerArray array_msg;
        array_msg.markers.push_back(msg);
        trajectory_vis_pub_.publish(array_msg);
        vis_completed_count_++;
    }

    void PlannerNode::publishEvalVisualization(const TrajectorySegment &trajectory) {
        // Visualize the gain of the current segment
        visualization_msgs::MarkerArray msg;
        trajectory_evaluator_->visualizeTrajectoryValue(&msg, trajectory);
        if (msg.markers.size() < vis_num_previous_evaluations_) {
            int temp_num_evals = msg.markers.size();
            // Make sure previous visualization is cleared again
            for (int i = msg.markers.size(); i < vis_num_previous_evaluations_; ++i) {
                // Setup marker message
                visualization_msgs::Marker new_msg;
                new_msg.header.frame_id = "/world";
                new_msg.header.stamp = ros::Time::now();
                new_msg.id = i;
                new_msg.ns = "evaluation";
                new_msg.action = visualization_msgs::Marker::DELETE;
                msg.markers.push_back(new_msg);
            }
            vis_num_previous_evaluations_ = temp_num_evals;
        }
        trajectory_vis_pub_.publish(msg);
    }

    bool PlannerNode::checkMinNewValue(const std::unique_ptr <TrajectorySegment> &segment) {
        // Recursively check wehter the minimum value is reached
        if (segment->value >= p_min_new_value_) {
            return true;
        } else {
            for (int i = 0; i < segment->children.size(); ++i) {
                if (checkMinNewValue(segment->children[i])) {
                    return true;
                }
            }
        }
        return false;
    }

    bool PlannerNode::runSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        res.success = true;
        if (req.data) {
            initializePlanning();
            running_ = true;
            ROS_INFO("Started planning.");
        } else {
            running_ = false;
            ROS_INFO("Stopped planning.");
        }
        return true;
    }

    bool PlannerNode::cpuSrvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        double time = (double) (std::clock() - cpu_srv_timer_) / CLOCKS_PER_SEC;
        cpu_srv_timer_ = std::clock();

        // Just return cpu time as the service message
        res.message = std::to_string(time).c_str();
        res.success = true;
        return true;
    }

} // namespace mav_active_3d_planning

int main(int argc, char **argv) {
    ros::init(argc, argv, "active_3d_planner");

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::PlannerNode planner_node(nh, nh_private);
    ROS_INFO("Initialized active_3d_planner_node.");
    planner_node.planningLoop();
}