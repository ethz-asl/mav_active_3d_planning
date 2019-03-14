#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluators.h"
#include "mav_active_3d_planning/module_factory.h"

#include <algorithm>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // SimpleYawPlanningEvaluator
        bool SimpleYawPlanningEvaluator::computeGain(TrajectorySegment *traj_in) {

        }

        bool SimpleYawPlanningEvaluator::computeCost(TrajectorySegment *traj_in) {
            return following_evaluator_->computeCost(traj_in);
        }

        bool SimpleYawPlanningEvaluator::computeValue(TrajectorySegment *traj_in) {
            return following_evaluator_->computeValue(traj_in);

        }

        int SimpleYawPlanningEvaluator::selectNextBest(const TrajectorySegment &traj_in) {
            return following_evaluator_->selectNextBest(traj_in);

        }

        bool SimpleYawPlanningEvaluator::updateSegments(TrajectorySegment *root) {
            return following_evaluator_->updateSegments(root);

        }

        void SimpleYawPlanningEvaluator::visualizeTrajectoryValue(visualization_msgs::Marker *msg,
                                                                  const TrajectorySegment &trajectory) {
            return following_evaluator_->visualizeTrajectoryValue(msg, trajectory);

        }

        void SimpleYawPlanningEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<int>(param_map, "n_directions", &p_n_directions_, 4);
            setParam<bool>(param_map, "select_by_value", &p_select_by_value_, false);

            // Create following evaluator
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_evaluator_args", &args,
                                  param_ns + "/following_evaluator");
            following_evaluator_ = ModuleFactory::Instance()->createTrajectoryEvaluator(args, voxblox_ptr_,
                                                                                        verbose_modules_);

            // setup parent
            TrajectoryEvaluator::setupFromParamMap(param_map);
        }

        bool SimpleYawPlanningEvaluator::checkParamsValid(std::string *error_message) {
            if (p_n_directions_ < 1) {
                *error_message = "n_directions expected > 0";
                return false;
            }
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
