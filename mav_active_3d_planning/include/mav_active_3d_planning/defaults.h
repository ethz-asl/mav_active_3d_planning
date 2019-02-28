#ifndef MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
#define MAV_ACTIVE_3D_PLANNING_DEFAULTS_H

#include "mav_active_3d_planning/trajectory_segment.h"

#include <Eigen/Core>
#include <ros/node_handle.h>

#include <functional>
#include <vector>
#include <string>

namespace mav_active_3d_planning {

    // Contains default methods and utility functions that can be used by various classes
    namespace defaults {

        // struct for bounding volumes (simple bounding box atm)
        struct BoundingVolume {
            BoundingVolume(std::string param_ns);

            virtual ~BoundingVolume() {}

            // check wether point is in bounding box, if bounding box is setup
            bool contains(Eigen::Vector3d point);

            bool is_setup;
            double x_min, x_max, y_min, y_max, z_min, z_max;
        };

        // set parameters from nodehandle and print config if requested
        template <typename T>
        void setParamFromRos(const ros::NodeHandle &nh, std::string param_name, T *param, bool verbose,
                bool(*isValid)(T) = nullptr){
            std::string appendix("");
            T value;
            if (!nh.getParam(param_name, value)) {
                appendix = " (default)";
            } else {
                if (isValid != nullptr) {
                    if (!(*isValid)(value)) {
                        ROS_WARN("Invalid value for parameter '%s', reverting to default.", param_name.c_str());
                        appendix = " (default)";
                    } else {
                        *param = value;
                    }
                } else {
                    *param = value;
                }
            }
            if (verbose) {
                ROS_INFO_STREAM(param_name << ": " << *param << appendix);
            }
        }

        // Return the indices of the EigenTrajectoryPoints in the segment to be evaluated
        std::vector<int> samplePointsFromSegment(const TrajectorySegment &segment, double sampling_rate,
                                                    bool include_first = false);

    } // namespace defaults
} // namepsace mav_active_3d_planning

#endif //MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
