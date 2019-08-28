#define _USE_MATH_DEFINES

#include "mav_active_3d_planning/modules/trajectory_evaluators/continuous_yaw_planning_evaluator.h"
#include "mav_active_3d_planning/module_factory.h"
#include "mav_active_3d_planning/defaults.h"

#include <cmath>
#include <vector>

namespace mav_active_3d_planning {
    namespace trajectory_evaluators {
        using YawPlanningUpdater = mav_active_3d_planning::evaluator_updaters::YawPlanningUpdater;

        ModuleFactory::Registration <ContinuousYawPlanningEvaluator> ContinuousYawPlanningEvaluator::registration(
                "ContinuousYawPlanningEvaluator");

        bool ContinuousYawPlanningEvaluator::computeGain(TrajectorySegment *traj_in) {
            // compute gain for all segments as in base class
            YawPlanningEvaluator::computeGain(traj_in);
            // find best stored yaw and apply it
            setBestYaw(traj_in);
        }

        bool ContinuousYawPlanningEvaluator::updateSegments(TrajectorySegment *root) {
            return updateSingle(root);
        }

        bool ContinuousYawPlanningEvaluator::updateSingle(TrajectorySegment *segment) {
            // recursively update the tree
            if (segment->parent && segment->info) {
                double dist = (planner_node_->getCurrentPosition() - segment->trajectory.back().position_W).norm();
                if (p_update_range_ == 0.0 || p_update_range_ > dist) {
                    bool update_all = (~p_update_sections_separate_) & (segment->gain > p_update_gain_);
                    YawPlanningInfo *info = dynamic_cast<YawPlanningInfo *>(segment->info.get());
                    for (int i = 0; i < info->orientations.size(); ++i) {
                        if (update_all || info->orientations[i].gain > p_update_gain_) {
                            // all conditions met: update gain of segment
                            following_evaluator_->computeGain(&(info->orientations[i]));
                        }
                    }
                }
                // Update trajectory
                setBestYaw(segment);
            }

            // propagate through tree
            for (int i = 0; i < segment->children.size(); ++i) {
                updateSingle(segment->children[i].get());
            }
            return true;
        }

        void ContinuousYawPlanningEvaluator::setBestYaw(TrajectorySegment *segment) {
            // compute gain from section overlap
            YawPlanningInfo *info = dynamic_cast<YawPlanningInfo *>(segment->info.get());
            int n_ori = info->orientations.size();
            std::vector<double> gains;
            for (int i = 0; i < n_ori; ++i) {
                double gain = info->orientations[i].gain;
                for (int j = 1; j < p_n_sections_fov_; ++j) {
                    gain += info->orientations[(i + j) % n_ori].gain;
                }
                gains.push_back(gain);
            }

            // find and apply best yaw
            double min_gain = gains[0];
            double max_gain = gains[0];
            int max_index = 0;
            for (int i = 1; i < gains.size(); ++i) {
                if (gains[i] > max_gain) {
                    max_gain = gains[i];
                    max_index = i;
                } else if (gains[i] < min_gain) {
                    min_gain = gains[i];
                }
            }
            double best_yaw;
            if (max_gain > min_gain || segment->trajectory.size() > 1) {
                // got a best yaw
                best_yaw = defaults::angleScaled(info->orientations[max_index].trajectory.back().getYaw() +
                           (double) p_n_sections_fov_ / p_n_directions_ * M_PI);
            } else {
                // indifferent, use direction of travel
                Eigen::Vector3d direction =
                        segment->trajectory.back().position_W - segment->trajectory.front().position_W;
                best_yaw = defaults::angleScaled(std::atan2(direction[1], direction[0]));
            }
            setTrajectoryYaw(segment, segment->trajectory.front().getYaw(), best_yaw);
            info->active_orientation = max_index;

            // always recompute cost and value, since trajectory can change
            segment->gain = 0.0;
            for (int j = 0; j < p_n_sections_fov_; ++j) {
                segment->gain += info->orientations[(max_index + j) % n_ori].gain;
            }
            following_evaluator_->computeCost(segment);
            following_evaluator_->computeValue(segment);
        }

        double ContinuousYawPlanningEvaluator::sampleYaw(double original_yaw, int sample_number) {
            // Uniform sampling
            return defaults::angleScaled(original_yaw + (double) sample_number * 2.0 * M_PI / p_n_directions_);
        }

        void
        ContinuousYawPlanningEvaluator::setTrajectoryYaw(TrajectorySegment *segment, double start_yaw,
                                                         double target_yaw) {
            // just set the yaw of the entire trajectory to the sampled value
            for (int i = 0; i < segment->trajectory.size(); ++i) {
                segment->trajectory[i].setFromYaw(target_yaw);
            }
        }

        void ContinuousYawPlanningEvaluator::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "visualize_followup", &p_visualize_followup_, true);
            setParam<int>(param_map, "n_sections_fov", &p_n_sections_fov_, 1);
            setParam<double>(param_map, "update_range", &p_update_range_, -1.0);    // default is no updates
            setParam<double>(param_map, "update_gain", &p_update_gain_, 0.0);
            setParam<bool>(param_map, "update_sections_separate", &p_update_sections_separate_, false);

            planner_node_ = dynamic_cast<PlannerNode *>(ModuleFactory::Instance()->readLinkableModule("PlannerNode"));

            // setup parent
            YawPlanningEvaluator::setupFromParamMap(param_map);
            p_select_by_value_ = false;     // Cannot select by value since views related
        }

        void ContinuousYawPlanningEvaluator::visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg,
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

                // Color according to relative value (blue when indifferent, grey for values below update range)
                if (info->orientations[i].gain <= p_update_sections_separate_) {
                    new_msg.color.r = 0.5;
                    new_msg.color.g = 0.5;
                    new_msg.color.b = 0.5;
                } else if (max_value != min_value) {
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
                for (int i = 0; i < p_n_sections_fov_; ++i) {
                    following_evaluator_->visualizeTrajectoryValue(msg,
                                                                   info->orientations[(info->active_orientation + i) %
                                                                                      p_n_directions_]);
                }
            }
        }

    } // namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
