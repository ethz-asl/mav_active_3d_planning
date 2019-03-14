#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_YAW_PLANNING_EVALUATORS_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_YAW_PLANNING_EVALUATORS_H

#include "mav_active_3d_planning/trajectory_evaluator.h"


namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace trajectory_evaluators {

        // Samples different yaws for the input trajectory and selects the best one. Evaluation and all other
        // functionalities are delegated to the folowing evaluator.
        class SimpleYawPlanningEvaluator : public TrajectoryEvaluator {
        public:

            // Override virtual functions
            bool computeGain(TrajectorySegment *traj_in);
            bool computeCost(TrajectorySegment *traj_in);
            bool computeValue(TrajectorySegment *traj_in);
            int selectNextBest(const TrajectorySegment &traj_in);
            bool updateSegments(TrajectorySegment *root);
            void visualizeTrajectoryValue(visualization_msgs::Marker* msg, const TrajectorySegment &trajectory);

        protected:
            friend ModuleFactory;

            // factory access
            SimpleYawPlanningEvaluator() {}

            void setupFromParamMap(Module::ParamMap *param_map);
            bool checkParamsValid(std::string *error_message);

            // members
            std::unique_ptr <TrajectoryEvaluator> following_evaluator_;

            // parameters
            int p_n_directions_;
            bool p_select_by_value_;    // false: only evaluate the gain, true: evaluate gain+cost+value
        };

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_EVALUATORS_YAW_PLANNING_EVALUATORS_H
