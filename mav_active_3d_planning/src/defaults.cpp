#include "mav_active_3d_planning/defaults.h"

#include <ros/param.h>

#include <random>
#include <algorithm>
#include <cmath>

namespace mav_active_3d_planning {
    namespace defaults {

        BoundingVolume::BoundingVolume(std::string param_ns) : is_setup(false) {
            ros::param::param<double>(param_ns + "/x_min", x_min, 0.0);
            ros::param::param<double>(param_ns + "/x_max", x_max, 0.0);
            ros::param::param<double>(param_ns + "/y_min", y_min, 0.0);
            ros::param::param<double>(param_ns + "/y_max", y_max, 0.0);
            ros::param::param<double>(param_ns + "/z_min", z_min, 0.0);
            ros::param::param<double>(param_ns + "/z_max", z_max, 0.0);
            if (x_max - x_min > 0.0 && y_max - y_min > 0.0 && z_max - z_min > 0.0) {
                is_setup = true;
            }
        }

        bool BoundingVolume::contains(Eigen::Vector3d point) {
            if (!is_setup) { return true; }
            if (point[0] < x_min) { return false; }
            if (point[0] > x_max) { return false; }
            if (point[1] < y_min) { return false; }
            if (point[1] > y_max) { return false; }
            if (point[2] < z_min) { return false; }
            if (point[2] > z_max) { return false; }
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
