#define _USE_MATH_DEFINES

#include "active_3d_planning_core/tools/defaults.h"

#include <cmath>

namespace active_3d_planning {
namespace defaults {

// Angle functions
double angleScaled(double angle) {
  angle = std::fmod(angle, 2.0 * M_PI);
  return angle + 2.0 * M_PI * (angle < 0);
}

double angleDifference(double angle1, double angle2) {
  double angle = std::abs(std::fmod(angle1 - angle2, 2.0 * M_PI));
  if (angle > M_PI) {
    angle = 2.0 * M_PI - angle;
  }
  return angle;
}

double angleDirection(double input, double target) {
  input = angleScaled(input);
  target = angleScaled(target);
  if (target == input) {
    return 0.0;
  }
  if (target > input) {
    if (target - input < M_PI) {
      return 1.0;
    } else {
      return -1.0;
    }
  } else {
    if (input - target < M_PI) {
      return -1.0;
    } else {
      return 1.0;
    }
  }
}

}  // namespace defaults
}  // namespace active_3d_planning
