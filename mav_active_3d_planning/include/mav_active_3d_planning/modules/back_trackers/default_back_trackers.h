#ifndef MAV_ACTIVE_3D_PLANNING_BACK_TRACKERS_DEFAULT_BACK_TRACKERS_H
#define MAV_ACTIVE_3D_PLANNING_BACK_TRACKERS_DEFAULT_BACK_TRACKERS_H

#include "mav_active_3d_planning/back_tracker.h"

#include <mav_msgs/eigen_mav_msgs.h>

#include <vector>

namespace mav_active_3d_planning {
    namespace back_trackers {

        // Try rotating in place until feasible trajectories were found
        class RotateInPlace : public BackTracker {
        public:
            // implement virtual functions
            bool trackBack(TrajectorySegment *target);

        protected:
            friend ModuleFactory;

            // factory access
            RotateInPlace() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);
            static ModuleFactory::Registration<RotateInPlace> registration;

            // params
            double turn_rate_;  // rad/s
            double update_rate_;   // Hz, determines length of rotation arc (after which we check for new trajectories)
            double sampling_rate_;  // Hz, trajectory is sampled with sample_rate
        };

        // Try rotating in place, if nothing found reverse most recent segments
        class RotateReverse : public BackTracker {
        public:
            // implement virtual functions
            bool segmentIsExecuted(const TrajectorySegment &segment);
            bool trackBack(TrajectorySegment *target);

        protected:
            friend ModuleFactory;

            // factory access
            RotateReverse() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);
            static ModuleFactory::Registration<RotateReverse> registration;

            // params
            double turn_rate_;  // rad/s
            double update_rate_;   // Hz (how often the turn movement is checked for new trajectories)
            double sampling_rate_;  // Hz
            double n_rotations_;
            int stack_size_;

            // variables
            std::vector<mav_msgs::EigenTrajectoryPointVector> stack_; // We use a vector for fixed stack size
            Eigen::Vector3d last_position_;
            double last_yaw_;
            double current_rotation_;

            // methods
            bool rotate(TrajectorySegment *target);
            bool reverse(TrajectorySegment *target);
        };

    } // namespace back_trackers
} // namepsace mav_active_3d_planning

#endif // MAV_ACTIVE_3D_PLANNING_BACK_TRACKERS_DEFAULT_BACK_TRACKERS_H

