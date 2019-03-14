#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluators.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <cmath>
#include <vector>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {

        // SimpleYawPlanningEvaluator
        bool SimpleYawPlanningEvaluator::computeGain(TrajectorySegment *traj_in) {
            double start_yaw = traj_in->trajectory.front().getYaw();
            double original_yaw = traj_in->trajectory.back().getYaw();
            std::vector <TrajectorySegment> segments;

            // Get value for initial position
            segments.push_back(traj_in->shallowCopy());
            setTrajectoryYaw(&(segments.back()), start_yaw, original_yaw);
            following_evaluator_->computeGain(&(segments.back()));
            double current_value = segments.back().gain;
            int best_index = 0;
            if (p_select_by_value_) {
                following_evaluator_->computeCost(&(segments.back()));
                following_evaluator_->computeValue(&(segments.back()));
                current_value = segments.back().value;
            }
            double best_value = current_value;

            // Sample all directions
            for (int i = 1; i < p_n_directions_; ++i) {
                segments.push_back(traj_in->shallowCopy());
                setTrajectoryYaw(&(segments.back()), start_yaw, sampleYaw(original_yaw, i));
                following_evaluator_->computeGain(&(segments.back()));
                if (p_select_by_value_) {
                    following_evaluator_->computeCost(&(segments.back()));
                    following_evaluator_->computeValue(&(segments.back()));
                    current_value = segments.back().value;
                } else {
                    current_value = segments.back().gain;
                }
                if (current_value > best_value) {
                    best_value = current_value;
                    best_index = i;
                }
            }

            // Select result and apply changes to original segment
            traj_in->trajectory = segments[best_index].trajectory;
            traj_in->gain = segments[best_index].gain;
            if (p_select_by_value_) {
                traj_in->cost = segments[best_index].cost;
                traj_in->value = segments[best_index].value;
            }
            traj_in->info = std::move(segments[best_index].info);
            return true;
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

        double SimpleYawPlanningEvaluator::sampleYaw(double original_yaw, int sampleNumber) {
            // Uniform sampling
            return defaults::angleScaled(original_yaw + (double) sampleNumber * 2.0 * M_PI / p_n_directions_);
        }

        void
        SimpleYawPlanningEvaluator::setTrajectoryYaw(TrajectorySegment *segment, double start_yaw, double target_yaw) {
            // just set the yaw of the entire trajectory atm
            for (int i = 0; i < segment->trajectory.size(); ++i) {
                segment->trajectory[i].setFromYaw(target_yaw);
            }
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
            if (p_n_directions_ < 2) {
                *error_message = "n_directions expected > 1";
                return false;
            }
            return true;
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
