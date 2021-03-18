#ifndef ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_H_
#define ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_H_

#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Eigen>

namespace active_3d_planning {

inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

struct EigenTrajectoryPoint {
  typedef std::vector<EigenTrajectoryPoint,
                      Eigen::aligned_allocator<EigenTrajectoryPoint>>
      Vector;

  EigenTrajectoryPoint()
      : timestamp_ns(-1),
        time_from_start_ns(0),
        position_W(Eigen::Vector3d::Zero()),
        velocity_W(Eigen::Vector3d::Zero()),
        acceleration_W(Eigen::Vector3d::Zero()),
        jerk_W(Eigen::Vector3d::Zero()),
        snap_W(Eigen::Vector3d::Zero()),
        orientation_W_B(Eigen::Quaterniond::Identity()),
        angular_velocity_W(Eigen::Vector3d::Zero()),
        angular_acceleration_W(Eigen::Vector3d::Zero()) {}

  EigenTrajectoryPoint(int64_t _time_from_start_ns,
                       const Eigen::Vector3d& _position,
                       const Eigen::Vector3d& _velocity,
                       const Eigen::Vector3d& _acceleration,
                       const Eigen::Vector3d& _jerk,
                       const Eigen::Vector3d& _snap,
                       const Eigen::Quaterniond& _orientation,
                       const Eigen::Vector3d& _angular_velocity,
                       const Eigen::Vector3d& _angular_acceleration)
      : timestamp_ns(-1),
        time_from_start_ns(_time_from_start_ns),
        position_W(_position),
        velocity_W(_velocity),
        acceleration_W(_acceleration),
        jerk_W(_jerk),
        snap_W(_snap),
        orientation_W_B(_orientation),
        angular_velocity_W(_angular_velocity),
        angular_acceleration_W(_angular_acceleration) {}

  EigenTrajectoryPoint(int64_t _time_from_start_ns,
                       const Eigen::Vector3d& _position,
                       const Eigen::Vector3d& _velocity,
                       const Eigen::Vector3d& _acceleration,
                       const Eigen::Vector3d& _jerk,
                       const Eigen::Vector3d& _snap,
                       const Eigen::Quaterniond& _orientation,
                       const Eigen::Vector3d& _angular_velocity)
      : EigenTrajectoryPoint(_time_from_start_ns, _position, _velocity,
                             _acceleration, _jerk, _snap, _orientation,
                             _angular_velocity, Eigen::Vector3d::Zero()) {}

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int64_t
      timestamp_ns;  // Time since epoch, negative value = invalid timestamp.
  int64_t time_from_start_ns;
  Eigen::Vector3d position_W;
  Eigen::Vector3d velocity_W;
  Eigen::Vector3d acceleration_W;
  Eigen::Vector3d jerk_W;
  Eigen::Vector3d snap_W;

  Eigen::Quaterniond orientation_W_B;
  Eigen::Vector3d angular_velocity_W;
  Eigen::Vector3d angular_acceleration_W;

  // Accessors for making dealing with orientation/angular velocity easier.
  inline double getYaw() const { return yawFromQuaternion(orientation_W_B); }

  inline double getYawRate() const { return angular_velocity_W.z(); }

  inline double getYawAcc() const { return angular_acceleration_W.z(); }

  // WARNING: sets roll and pitch to 0.
  inline void setFromYaw(double yaw) {
    orientation_W_B = quaternionFromYaw(yaw);
  }

  inline void setFromYawRate(double yaw_rate) {
    angular_velocity_W.x() = 0.0;
    angular_velocity_W.y() = 0.0;
    angular_velocity_W.z() = yaw_rate;
  }

  inline void setFromYawAcc(double yaw_acc) {
    angular_acceleration_W.x() = 0.0;
    angular_acceleration_W.y() = 0.0;
    angular_acceleration_W.z() = yaw_acc;
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "position:          " << position_W.transpose() << std::endl
       << "velocity:          " << velocity_W.transpose() << std::endl
       << "acceleration:      " << acceleration_W.transpose() << std::endl
       << "jerk:              " << jerk_W.transpose() << std::endl
       << "snap:              " << snap_W.transpose() << std::endl
       << "yaw:               " << getYaw() << std::endl
       << "yaw_rate:          " << getYawRate() << std::endl
       << "yaw_acc:           " << getYawAcc() << std::endl;

    return ss.str();
  }
};

// Operator overload to transform Trajectory Points according to the Eigen
// interfaces (uses operator* for this).
// Has to be outside of class.
// Example:
// Eigen::Affine3d transform; EigenTrajectoryPoint point;
// EigenTrajectoryPoint transformed = transform * point;
inline EigenTrajectoryPoint operator*(const Eigen::Affine3d& lhs,
                                      const EigenTrajectoryPoint& rhs) {
  EigenTrajectoryPoint transformed(rhs);
  transformed.position_W = lhs * rhs.position_W;
  transformed.velocity_W = lhs.rotation() * rhs.velocity_W;
  transformed.acceleration_W = lhs.rotation() * rhs.acceleration_W;
  transformed.jerk_W = lhs.rotation() * rhs.jerk_W;
  transformed.snap_W = lhs.rotation() * rhs.snap_W;
  transformed.orientation_W_B = lhs.rotation() * rhs.orientation_W_B;
  transformed.angular_velocity_W = lhs.rotation() * rhs.angular_velocity_W;
  transformed.angular_acceleration_W =
      lhs.rotation() * rhs.angular_acceleration_W;
  return transformed;
}

// TODO(huberya): clean this up
#define MAV_MSGS_CONCATENATE(x, y) x##y
#define MAV_MSGS_CONCATENATE2(x, y) MAV_MSGS_CONCATENATE(x, y)
#define MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EIGEN_TYPE)                    \
  typedef std::vector<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE>> \
      MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Vector);                        \
  typedef std::deque<EIGEN_TYPE, Eigen::aligned_allocator<EIGEN_TYPE>>  \
      MAV_MSGS_CONCATENATE2(EIGEN_TYPE, Deque);

MAV_MSGS_MAKE_ALIGNED_CONTAINERS(EigenTrajectoryPoint)

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_H_
