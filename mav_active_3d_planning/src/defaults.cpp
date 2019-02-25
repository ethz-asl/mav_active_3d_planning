#define _USE_MATH_DEFINES

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

        double angleScaled(double angle){
            angle = std::fmod(angle, 2.0 * M_PI);
            return angle + 2.0 * M_PI * (angle < 0);
        }

        double angleDifference(double angle1, double angle2){
            double angle = std::abs(std::fmod(angle1 - angle2, 2.0 * M_PI));
            if (angle > M_PI){
                angle = 2.0 * M_PI - angle;
            }
            return angle;
        }

        double angleDirection(double input, double target){
            input = angleScaled(input);
            target = angleScaled(target);
            if (target == input) { return 0.0; }
            if (target > input){
                if (target - input < M_PI){
                    return 1.0;
                } else {
                    return -1.0;
                }
            } else {
                if (input - target < M_PI){
                    return -1.0;
                } else {
                    return 1.0;
                }
            }
        }

    } // namespace defaults
} // namepsace mav_active_3d_planning
