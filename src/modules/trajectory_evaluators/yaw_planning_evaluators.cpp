#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_evaluators/yaw_planning_evaluators.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <cmath>
#include <vector>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {
        using YawPlanningUpdater = mav_active_3d_planning::evaluator_updaters::YawPlanningUpdater;

        // YawPlanningEvaluator
        bool YawPlanningEvaluator::computeGain(TrajectorySegment *traj_in) {
            // Init
            double start_yaw = traj_in->trajectory.front().getYaw();
            double original_yaw = traj_in->trajectory.back().getYaw();
            YawPlanningInfo *info = new YawPlanningInfo();   // already create the info struct and directly populate it
            info->active_orientation = 0;

            // Get value for initial position
            info->orientations.push_back(traj_in->shallowCopy());
            setTrajectoryYaw(&(info->orientations.back()), start_yaw, original_yaw);
            following_evaluator_->computeGain(&(info->orientations.back()));
            double current_value = info->orientations.back().gain;
            if (p_select_by_value_) {
                following_evaluator_->computeCost(&(info->orientations.back()));
                following_evaluator_->computeValue(&(info->orientations.back()));
                current_value = info->orientations.back().value;
            }
            double best_value = current_value;
            // Sample all directions
            for (int i = 1; i < p_n_directions_; ++i) {
                info->orientations.push_back(traj_in->shallowCopy());
                setTrajectoryYaw(&(info->orientations.back()), start_yaw, sampleYaw(original_yaw, i));
                following_evaluator_->computeGain(&(info->orientations.back()));
                if (p_select_by_value_) {
                    following_evaluator_->computeCost(&(info->orientations.back()));
                    following_evaluator_->computeValue(&(info->orientations.back()));
                    current_value = info->orientations.back().value;
                } else {
                    current_value = info->orientations.back().gain;
                }
                if (current_value > best_value) {
                    best_value = current_value;
                    info->active_orientation = i;
                }
            }

            // Apply best segment to the original one, store rest in info
            traj_in->trajectory = info->orientations[info->active_orientation].trajectory;
            traj_in->gain = info->orientations[info->active_orientation].gain;
            if (p_select_by_value_) {
                traj_in->cost = info->orientations[info->active_orientation].cost;
                traj_in->value = info->orientations[info->active_orientation].value;
            }
            traj_in->info.reset(info);
            return true;
        }

        bool YawPlanningEvaluator::computeCost(TrajectorySegment *traj_in) {
            return following_evaluator_->computeCost(traj_in);
        }

        bool YawPlanningEvaluator::computeValue(TrajectorySegment *traj_in) {
            return following_evaluator_->computeValue(traj_in);
        }

        int YawPlanningEvaluator::selectNextBest(TrajectorySegment *traj_in) {
            return following_evaluator_->selectNextBest(traj_in);
        }

        bool YawPlanningEvaluator::updateSegments(TrajectorySegment *root) {
            return following_evaluator_->updateSegments(root);
        }

        void YawPlanningEvaluator::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
                                                            const TrajectorySegment &trajectory) {
            if (!trajectory.info) { return; }
            YawPlanningInfo *info = dynamic_cast<YawPlanningInfo *>(trajectory.info.get());
            // Let the followup evaluator draw the visualization of the selected orientation
            following_evaluator_->visualizeTrajectoryValue(msg, info->orientations[info->active_orientation]);
        }

        void YawPlanningEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<int>(param_map, "n_directions", &p_n_directions_, 4);
            setParam<bool>(param_map, "select_by_value", &p_select_by_value_, false);

            // Register link for yaw planning udpaters
            ModuleFactory::Instance()->registerLinkableModule("YawPlanningEvaluator", this);

            // Create following evaluator
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_evaluator_args", &args,
                                  param_ns + "/following_evaluator");
            following_evaluator_ = ModuleFactory::Instance()->createModule<TrajectoryEvaluator>(args, verbose_modules_);

            // setup parent
            TrajectoryEvaluator::setupFromParamMap(param_map);
        }

        bool YawPlanningEvaluator::checkParamsValid(std::string *error_message) {
            if (p_n_directions_ < 2) {
                *error_message = "n_directions expected > 1";
                return false;
            }
            return TrajectoryEvaluator::checkParamsValid(error_message);
        }

        // SimpleYawPlanningEvaluator
        ModuleFactory::Registration <SimpleYawPlanningEvaluator> SimpleYawPlanningEvaluator::registration(
                "SimpleYawPlanningEvaluator");

        double SimpleYawPlanningEvaluator::sampleYaw(double original_yaw, int sample_number) {
            // Uniform sampling
            return defaults::angleScaled(original_yaw + (double) sample_number * 2.0 * M_PI / p_n_directions_);
        }

        void
        SimpleYawPlanningEvaluator::setTrajectoryYaw(TrajectorySegment *segment, double start_yaw, double target_yaw) {
            // just set the yaw of the entire trajectory to the sampled value
            for (int i = 0; i < segment->trajectory.size(); ++i) {
                segment->trajectory[i].setFromYaw(target_yaw);
            }
        }

        void SimpleYawPlanningEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "visualize_followup", &p_visualize_followup_, true);
            // setup parent
            YawPlanningEvaluator::setupFromParamMap(param_map);
        }

        void SimpleYawPlanningEvaluator::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
                                                                  const TrajectorySegment &trajectory) {
            if (!trajectory.info) { return; }
            YawPlanningInfo *info = dynamic_cast<YawPlanningInfo *>(trajectory.info.get());
            // Simple version: visualize facing of endpoints of trajectories, colored with value (highest green to lowest red)
            double max_value = info->orientations[0].gain;
            double min_value = info->orientations[0].gain;
            if (p_select_by_value_) {
                max_value = info->orientations[0].value;
                min_value = info->orientations[0].value;
            }
            for (int i = 1; i < info->orientations.size(); ++i) {
                if (p_select_by_value_) {
                    if (info->orientations[i].value > max_value) {
                        max_value = info->orientations[i].value;
                    }
                    if (info->orientations[i].value < min_value) {
                        min_value = info->orientations[i].value;
                    }
                } else {
                    if (info->orientations[i].gain > max_value) {
                        max_value = info->orientations[i].gain;
                    }
                    if (info->orientations[i].gain < min_value) {
                        min_value = info->orientations[i].gain;
                    }
                }
            }
            for (int i = 0; i < info->orientations.size(); ++i) {
                // Setup marker message
                visualization_msgs::Marker new_msg;
                new_msg.header.frame_id = "/world";
                new_msg.ns = "evaluation";
                new_msg.header.stamp = ros::Time::now();
                new_msg.pose.position.x = info->orientations[i].trajectory.back().position_W.x();
                new_msg.pose.position.y = info->orientations[i].trajectory.back().position_W.y();
                new_msg.pose.position.z = info->orientations[i].trajectory.back().position_W.z();
                new_msg.pose.orientation.x = info->orientations[i].trajectory.back().orientation_W_B.x();
                new_msg.pose.orientation.y = info->orientations[i].trajectory.back().orientation_W_B.y();
                new_msg.pose.orientation.z = info->orientations[i].trajectory.back().orientation_W_B.z();
                new_msg.pose.orientation.w = info->orientations[i].trajectory.back().orientation_W_B.w();
                new_msg.type = visualization_msgs::Marker::ARROW;
                new_msg.id = defaults::getNextVisualizationId(*msg);
                new_msg.scale.x = 0.6;
                new_msg.scale.y = 0.07;
                new_msg.scale.z = 0.07;
                new_msg.action = visualization_msgs::Marker::ADD;

                // Color according to relative value (blue when indifferent)
                if (max_value != min_value) {
                    double frac = (info->orientations[i].gain - min_value) / (max_value - min_value);
                    if (p_select_by_value_) {
                        frac = (info->orientations[i].value - min_value) / (max_value - min_value);
                    }
                    new_msg.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
                    new_msg.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
                    new_msg.color.b = 0.0;
                } else {
                    new_msg.color.r = 0.3;
                    new_msg.color.g = 0.3;
                    new_msg.color.b = 1.0;
                }
                new_msg.color.a = 0.4;
                msg->markers.push_back(new_msg);
            }

            // Followup
            if (p_visualize_followup_) {
                following_evaluator_->visualizeTrajectoryValue(msg, info->orientations[info->active_orientation]);
            }
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
