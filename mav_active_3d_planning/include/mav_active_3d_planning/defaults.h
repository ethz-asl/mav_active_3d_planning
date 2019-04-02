#ifndef MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
#define MAV_ACTIVE_3D_PLANNING_DEFAULTS_H

#include "mav_active_3d_planning/trajectory_segment.h"
#include "mav_active_3d_planning/module_factory.h"

#include <Eigen/Core>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <string>

namespace mav_active_3d_planning {

    // Contains default methods and utility functions that can be used by various classes
    namespace defaults {

        // struct for bounding volumes (simple box atm, might include z-rotation, sphere, set of planes, ...)
        struct BoundingVolume : public Module {
            BoundingVolume() : is_setup(false) {}

            virtual ~BoundingVolume() {}

            // factory parametrization
            void setupFromFactory(std::string args, bool verbose);

            // populate the bounding volume
            void setupFromParamMap(Module::ParamMap *param_map);

            // check wether point is in bounding box, if bounding box is setup
            bool contains(const Eigen::Vector3d &point);

            // variables
            bool is_setup;
            double x_min, x_max, y_min, y_max, z_min, z_max;

        protected:
            static ModuleFactory::Registration<BoundingVolume> registration;
        };

        // struct for high level system constraints (might add methods for generating these from max thrusts or so)
        struct SystemConstraints : public Module {
            SystemConstraints() {}

            virtual ~SystemConstraints() {}

            // factory constructor
            SystemConstraints(const std::string &args);

            // populate the system constraints from factory
            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);
            
            // variables
            double v_max;           // m/s, maximum absolute velocity
            double a_max ;          // m/s2, maximum absolute acceleration
            double yaw_rate_max;    // rad/s, maximum yaw rate

        protected:
            static ModuleFactory::Registration<SystemConstraints> registration;
        };

        // Scale an angle to [0, 2pi]
        double angleScaled(double angle);

        // Compute the difference between two angles in rad
        double angleDifference(double angle1, double angle2);

        // Returns the rotation direction (+1 / -1) that is closer for two angles
        double angleDirection(double angle1, double angle2);

        // Get the current next id for visualization markerarray messages
        int getNextVisualizationId(const visualization_msgs::MarkerArray &msg);

    } // namespace defaults

} // namepsace mav_active_3d_planning

#endif //MAV_ACTIVE_3D_PLANNING_DEFAULTS_H
