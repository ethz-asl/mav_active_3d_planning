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

        // struct for bounding volumes (simple box atm, might include z-rotation, sphere, set of planes, ...)
        struct BoundingVolume {
            BoundingVolume() : is_setup(false) {}

            BoundingVolume(std::string param_ns) {
                ros::NodeHandle nh(param_ns);
                setupFromRos(nh);
            }

            virtual ~BoundingVolume() {}

            // populate the bounding volume from ros params
            bool setupFromRos(const ros::NodeHandle &nh);

            // check wether point is in bounding box, if bounding box is setup
            bool contains(const Eigen::Vector3d &point);

            // variables
            bool is_setup;
            double x_min, x_max, y_min, y_max, z_min, z_max;
        };

        // struct for high level system constraints (might add methods for generating these from max thrusts or so)
        struct SystemConstraints {
            SystemConstraints() : is_setup(false) {}

            SystemConstraints(std::string param_ns) {
                ros::NodeHandle nh(param_ns);
                setupFromRos(nh);
            }

            virtual ~SystemConstraints() {}

            // populate the system constraints from ros params
            bool setupFromRos(const ros::NodeHandle &nh);

            // populate the system constraints with reasonable (conservative) defaults
            bool setupFromDefaults();

            // variables
            bool is_setup;
            double v_max;           // m/s, maximum absolute velocity
            double a_max ;          // m/s2, maximum absolute acceleration
            double yaw_rate_max;    // rad/s, maximum yaw rate
        };

        // Return the indices of the EigenTrajectoryPoints in the segment to be evaluated
        std::vector<int> samplePointsFromSegment(const TrajectorySegment &segment, double sampling_rate,
                                                    bool include_first = false);

    } // namespace defaults
} // namepsace mav_active_3d_planning

#endif //MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
