#include "mav_active_3d_planning/modules/trajectory_generators/linear_mav_trajectory_generator.h"
#include "mav_active_3d_planning/defaults.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <vector>



namespace mav_active_3d_planning {

    class AEPTrajectoryAdapter {
    public:
        AEPTrajectoryAdapter(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

        virtual ~AEPTrajectoryAdapter() {}

        // ros callbacks
        void poseCallback(const geometry_msgs::PoseStamped &msg);
        void trajCallback(const geometry_msgs::TwistStamped &msg);
    protected:
        // ros
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Subscriber pose_sub_;
        ros::Subscriber traj_sub_;
        ros::Publisher traj_pub_;
        ros::Publisher vis_pub_;

        // variables
        Eigen::Vector3d current_position_;  // Current pose of the mav
        double current_yaw_;

        double projection_time_ = 0.1;
        double sampling_rate_ = 0.01;

        Eigen::Vector3d goal_pos_;
        double goal_yaw_;
        bool goal_reached_;
        int vis_completed_count_;

    };

    AEPTrajectoryAdapter::AEPTrajectoryAdapter(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
            : nh_(nh),
              nh_private_(nh_private),
              goal_reached_(true),
              vis_completed_count_(0) {
        traj_pub_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
                mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
        vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/planner/trajectory_visualization", 100);

        pose_sub_ = nh_.subscribe("pose_in", 1, &AEPTrajectoryAdapter::poseCallback, this);
        traj_sub_ = nh_.subscribe("vel_in", 1, &AEPTrajectoryAdapter::trajCallback, this);
    }

    void AEPTrajectoryAdapter::poseCallback(const geometry_msgs::PoseStamped &msg) {
        // Track the current pose

        Eigen::Vector3d pos = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y,
                                              msg.pose.position.z);
        if (goal_reached_){
            // init
            goal_pos_ = pos;
            goal_reached_ = false;
        }
        Eigen::Vector3d dir = pos-goal_pos_;
        double dist = dir.norm();
        if (dist > 0.05) {
            visualization_msgs::Marker msg_vis;
            msg_vis.header.frame_id = "/world";
            msg_vis.header.stamp = ros::Time::now();
            msg_vis.pose.orientation.w = 1.0;
            msg_vis.type = visualization_msgs::Marker::POINTS;
            msg_vis.ns = "completed_trajectory";
            msg_vis.id = vis_completed_count_;
            msg_vis.action = visualization_msgs::Marker::ADD;
            msg_vis.scale.x = 0.1;
            msg_vis.scale.y = 0.1;
            msg_vis.color.r = 1.0;
            msg_vis.color.g = 0.0;
            msg_vis.color.b = 0.0;
            msg_vis.color.a = 1.0;

            // points
            double d = 0;
            Eigen::Vector3d x;
            while (d <= dist) {
                x = goal_pos_ + d/dist*dir;
                geometry_msgs::Point point;
                point.x = x[0];
                point.y = x[1];
                point.z = x[2];
                msg_vis.points.push_back(point);
                d += 0.05;
            }
            visualization_msgs::MarkerArray array_msg;
            array_msg.markers.push_back(msg_vis);
            vis_pub_.publish(array_msg);
            vis_completed_count_++;
            goal_pos_ = pos;
        }
        current_position_ = pos;

        current_yaw_ = tf::getYaw(msg.pose.orientation);
        // vis
    }

    void AEPTrajectoryAdapter::trajCallback(const geometry_msgs::TwistStamped &msg) {

        mav_msgs::EigenTrajectoryPoint start;
        mav_msgs::EigenTrajectoryPointVector result;

        // set goal
        double t = 0.0;     // seconds
        Eigen::Vector3d twist_lin(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);

        while (t < projection_time_) {
            start = mav_msgs::EigenTrajectoryPoint();
            start.position_W = current_position_ + t * twist_lin;
            start.velocity_W = twist_lin;
            start.setFromYaw(current_yaw_ + t * msg.twist.angular.z);
            start.setFromYawRate(msg.twist.angular.z);
            start.time_from_start_ns = static_cast<int64_t>(t * 1.0e9);
            result.push_back(start);
            t += sampling_rate_;
        }

        trajectory_msgs::MultiDOFJointTrajectoryPtr msg_out(new trajectory_msgs::MultiDOFJointTrajectory);
        msg_out->header.stamp = msg.header.stamp;
        int n_points = result.size();
        msg_out->points.resize(n_points);
        for (int i = 0; i < n_points; ++i) {
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(result[i], &msg_out->points[i]);
        }
        traj_pub_.publish(msg_out);
    }

} // namespace mav_active_3d_planning


int main(int argc, char **argv) {
    ros::init(argc, argv, "AEP_trajectory_adapter");

    // Set logging to debug for testing
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::AEPTrajectoryAdapter adaptor(nh, nh_private);
    ROS_INFO("Initialized AEP_trajectory_adapter.");
    ros::spin();
}