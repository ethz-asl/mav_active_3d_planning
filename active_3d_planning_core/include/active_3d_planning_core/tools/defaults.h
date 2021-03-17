#ifndef ACTIVE_3D_PLANNING_CORE_TOOLS_DEFAULTS_H_
#define ACTIVE_3D_PLANNING_CORE_TOOLS_DEFAULTS_H_

namespace active_3d_planning {

// Contains default methods and utility functions that can be used by various
// classes
namespace defaults {

// Scale an angle to [0, 2pi]
double angleScaled(double angle);

// Compute the closest positive difference between two angles in rad
double angleDifference(double angle1, double angle2);

// Returns the rotation direction (+1 / -1) that is closer for two angles
double angleDirection(double input, double target);

}  // namespace defaults
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_TOOLS_DEFAULTS_H_
