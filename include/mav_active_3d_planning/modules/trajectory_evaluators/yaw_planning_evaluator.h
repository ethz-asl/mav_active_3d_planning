#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_YAW_PLANNING_EVALUATOR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_YAW_PLANNING_EVALUATOR_H

#include "mav_active_3d_planning/trajectory_evaluator.h"


namespace mav_active_3d_planning {
    namespace evaluator_updaters { class YawPlanningUpdater; }
    namespace trajectory_evaluators {

        // Base class for yaw adapting evaluators. Evaluates different yaws for the input trajectory and selects the
        // best one. Evaluation and all other functionalities are delegated to the folowing evaluator.
        class YawPlanningEvaluator : public TrajectoryEvaluator {
        public:

            // Override virtual functions
            bool computeGain(TrajectorySegment *traj_in);
            bool computeCost(TrajectorySegment *traj_in);
            bool computeValue(TrajectorySegment *traj_in);
            int selectNextBest(TrajectorySegment *traj_in);
            bool updateSegments(TrajectorySegment *root);
            virtual void visualizeTrajectoryValue(visualization_msgs::MarkerArray* msg, const TrajectorySegment &trajectory);

        protected:
            friend ModuleFactory;
            friend mav_active_3d_planning::evaluator_updaters::YawPlanningUpdater;

            // factory access
            YawPlanningEvaluator() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);

            // members
            std::unique_ptr <TrajectoryEvaluator> following_evaluator_;

            // parameters
            int p_n_directions_;
            bool p_select_by_value_;        // false: only evaluate the gain, true: evaluate gain+cost+value

            // methods
            virtual double sampleYaw(double original_yaw, int sample) = 0;
            virtual void setTrajectoryYaw(TrajectorySegment* segment, double start_yaw, double target_yaw) = 0;
        };

        // Information struct that is assigned to segments
        struct YawPlanningInfo : public TrajectoryInfo {
            virtual ~YawPlanningInfo() {}

            std::vector <TrajectorySegment> orientations;   // Keep all orientated segment versions stored
            int active_orientation;
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_YAW_PLANNING_EVALUATOR_H
