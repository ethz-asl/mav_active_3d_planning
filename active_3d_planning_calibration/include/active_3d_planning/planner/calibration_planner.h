#ifndef ACTIVE_3D_PLANNING_CALIBRATION_CALIBRATION_PLANNER_H_
#define ACTIVE_3D_PLANNING_CALIBRATION_CALIBRATION_PLANNER_H_

#include "active_3d_planning/planner/ros_planner.h"

namespace active_3d_planning {
namespace ros {

class CalibrationPlanner : public RosPlanner {
public:
  CalibrationPlanner(const ::ros::NodeHandle &nh,
                     const ::ros::NodeHandle &nh_private);

  virtual ~CalibrationPlanner() = default;

protected:
  // Start/Run the planning loop
  void loopIteration() override;
};
} // namespace ros
} // namespace active_3d_planning

#endif // ACTIVE_3D_PLANNING_CALIBRATION_CALIBRATION_PLANNER_H_
