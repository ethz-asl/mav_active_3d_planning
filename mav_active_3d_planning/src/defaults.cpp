#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/defaults.h"

#include <ros/console.h>
#include <ros/param.h>

#include <random>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace defaults {

        bool BoundingVolume::setupFromRos(const ros::NodeHandle &nh){
            nh.param<double>("x_min", x_min, 0.0);
            nh.param<double>("x_max", x_max, 0.0);
            nh.param<double>("y_min", y_min, 0.0);
            nh.param<double>("y_max", y_max, 0.0);
            nh.param<double>("z_min", z_min, 0.0);
            nh.param<double>("z_max", z_max, 0.0);

            // Check params span a valid volume
            if (x_max > x_min && y_max > y_min && z_max > z_min) {
                is_setup = true;
            } else {
                is_setup = false;
                ROS_WARN("Bounding volume set up from '%s' does not span a valid volume.", nh.getNamespace().c_str());
            }
            return is_setup;
        }

        bool BoundingVolume::contains(const Eigen::Vector3d &point) {
            if (!is_setup) { return true; } // Uninitialized boxes always return true, so can always check them
            if (point.x() < x_min) { return false; }
            if (point.x() > x_max) { return false; }
            if (point.y() < y_min) { return false; }
            if (point.y() > y_max) { return false; }
            if (point.z() < z_min) { return false; }
            if (point.z() > z_max) { return false; }
            return true;
        }

        bool SystemConstraints::setupFromRos(const ros::NodeHandle &nh){
            setupFromDefaults();    // Default initialization
            nh.getParam("v_max", v_max);
            nh.getParam("a_max", a_max);
            nh.getParam("yaw_rate_max", yaw_rate_max);

            // Check params reasonable
            if (v_max > 0.0 && a_max > 0.0 && yaw_rate_max > 0.0) {
                is_setup = true;
            } else {
                is_setup = false;
                ROS_WARN("System constraints set up from '%s' are not valid.", nh.getNamespace().c_str());
            }
            return is_setup;
        }

        bool SystemConstraints::setupFromDefaults(){
            v_max = 1.0;
            a_max = 1.0;
            yaw_rate_max = M_PI / 2.0;
            is_setup = true;
            return true;
        }

        std::vector<int> samplePointsFromSegment(const TrajectorySegment &segment, double sampling_rate, bool include_first) {
            std::vector<int> result;
            if (sampling_rate <= 0.0) {
                // Rate of 0 means only last point
                result.push_back(segment.trajectory.size()-1);
                return result;
            }
            int64_t sampling_time_ns = static_cast<int64_t>(sampling_rate * 1.0e9);
            if (include_first && segment.trajectory.front().time_from_start_ns < sampling_time_ns) {
                result.push_back(0);
            }
            if (sampling_time_ns >= segment.trajectory.back().time_from_start_ns) {
                // no points within one sampling interval: add last point
                result.push_back(segment.trajectory.size()-1);
                return result;
            }
            int64_t current_time = sampling_time_ns;
            for (int i = 0; i < segment.trajectory.size(); ++i) {
                if (segment.trajectory[i].time_from_start_ns >= current_time) {
                    current_time += sampling_time_ns;
                    result.push_back(i);
                }
            }
            return result;
        }
    } // namespace defaults
} // namepsace mav_active_3d_planning
