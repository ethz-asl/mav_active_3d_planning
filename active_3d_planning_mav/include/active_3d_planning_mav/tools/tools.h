#ifndef ACTIVE_3D_PLANNING_MAV_TOOLS_TOOLS_H_
#define ACTIVE_3D_PLANNING_MAV_TOOLS_TOOLS_H_

#include <mav_msgs/eigen_mav_msgs.h>

#include "active_3d_planning_core/data/trajectory.h"

namespace active_3d_planning {

// not really clean solution but works I guess (and is probably ~0 cost)
inline const ::mav_msgs::EigenTrajectoryPointVector& planningMsgToMavMsg(
    const EigenTrajectoryPointVector& planningTraj) {
  return *reinterpret_cast<const ::mav_msgs::EigenTrajectoryPointVector*>(
      &planningTraj);
}

inline ::mav_msgs::EigenTrajectoryPointVector& planningMsgToMavMsg(
    EigenTrajectoryPointVector* planningTraj) {
  CHECK_NOTNULL(planningTraj);
  return *reinterpret_cast<::mav_msgs::EigenTrajectoryPointVector*>(
      planningTraj);
}

inline const EigenTrajectoryPointVector& mavMsgToPlanningMsg(
    const ::mav_msgs::EigenTrajectoryPointVector& mavTraj) {
  return *reinterpret_cast<const EigenTrajectoryPointVector*>(&mavTraj);
}

inline EigenTrajectoryPointVector& mavMsgToPlanningMsg(
    ::mav_msgs::EigenTrajectoryPointVector* mavTraj) {
  CHECK_NOTNULL(mavTraj);
  return *reinterpret_cast<EigenTrajectoryPointVector*>(mavTraj);
}

}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_MAV_TOOLS_TOOLS_H_
