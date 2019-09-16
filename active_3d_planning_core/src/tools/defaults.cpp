#define _USE_MATH_DEFINES

#include "active_3d_planning/tools/defaults.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace active_3d_planning {
namespace defaults {

// BoundingVolume
ModuleFactoryRegistry::Registration<BoundingVolume>
    BoundingVolume::registration("BoundingVolume");

BoundingVolume::BoundingVolume(PlannerI &planner)
    : Module(planner), is_setup(false) {}

void BoundingVolume::setupFromParamMap(Module::ParamMap *param_map) {
  setParam<double>(param_map, "x_min", &x_min, 0.0);
  setParam<double>(param_map, "x_max", &x_max, 0.0);
  setParam<double>(param_map, "y_min", &y_min, 0.0);
  setParam<double>(param_map, "y_max", &y_max, 0.0);
  setParam<double>(param_map, "z_min", &z_min, 0.0);
  setParam<double>(param_map, "z_max", &z_max, 0.0);
  setParam<double>(param_map, "rotation", &rotation, 0.0);

  rotation_quat = Eigen::AngleAxis<double>(rotation * M_PI / -180.0,
                                           Eigen::Vector3d::UnitZ());

  // Check params span a valid volume (This is not a compulsory check to allow
  // the unspecified (default) case, where the volume is not checked)
  if (x_max > x_min && y_max > y_min && z_max > z_min) {
    is_setup = true;
  } else {
    is_setup = false;
  }
}

bool BoundingVolume::contains(const Eigen::Vector3d &point) {
  if (!is_setup) {
    return true;
  } // Uninitialized boxes always return true, so can check them by default
  Eigen::Vector3d rotated_point = rotation_quat * point;
  if (rotated_point.x() < x_min) {
    return false;
  }
  if (rotated_point.x() > x_max) {
    return false;
  }
  if (rotated_point.y() < y_min) {
    return false;
  }
  if (rotated_point.y() > y_max) {
    return false;
  }
  if (rotated_point.z() < z_min) {
    return false;
  }
  if (rotated_point.z() > z_max) {
    return false;
  }
  return true;
}

// SystemConstraints
ModuleFactoryRegistry::Registration<SystemConstraints>
    SystemConstraints::registration("SystemConstraints");

SystemConstraints::SystemConstraints(PlannerI& planner) : Module(planner){}

void SystemConstraints::setupFromParamMap(Module::ParamMap *param_map) {
  setParam<double>(param_map, "v_max", &v_max, 1.0);
  setParam<double>(param_map, "a_max", &a_max, 1.0);
  setParam<double>(param_map, "yaw_rate_max", &yaw_rate_max, M_PI / 2.0);
}

bool SystemConstraints::checkParamsValid(std::string *error_message) {
  if (v_max <= 0.0) {
    *error_message = "v_max expected > 0.0";
    return false;
  } else if (a_max <= 0.0) {
    *error_message = "a_max expected > 0.0";
    return false;
  } else if (yaw_rate_max <= 0.0) {
    *error_message = "yaw_rate_max expected > 0.0";
    return false;
  }
  return true;
}

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

} // namespace defaults
} // namespace mav_active_3d_planning
