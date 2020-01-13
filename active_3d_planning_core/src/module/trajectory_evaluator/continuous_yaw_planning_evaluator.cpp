#define _USE_MATH_DEFINES

#include "active_3d_planning_core/module/trajectory_evaluator/continuous_yaw_planning_evaluator.h"

#include "active_3d_planning_core/planner/planner_I.h"
#include "active_3d_planning_core/tools/defaults.h"

#include <cmath>
#include <vector>

namespace active_3d_planning {
    namespace trajectory_evaluator {

        using YawPlanningUpdater =
        active_3d_planning::evaluator_updater::YawPlanningUpdater;

        ModuleFactoryRegistry::Registration<ContinuousYawPlanningEvaluator>
                ContinuousYawPlanningEvaluator::registration(
                "ContinuousYawPlanningEvaluator");

        ContinuousYawPlanningEvaluator::ContinuousYawPlanningEvaluator(
                PlannerI &planner)
                : YawPlanningEvaluator(planner) {}

        void ContinuousYawPlanningEvaluator::setupFromParamMap(
                Module::ParamMap *param_map) {
            setParam<bool>(param_map, "visualize_followup", &p_visualize_followup_, true);
            setParam<int>(param_map, "n_sections_fov", &p_n_sections_fov_, 1);
            setParam<double>(param_map, "update_range", &p_update_range_,
                             -1.0); // default is no updates
            setParam<double>(param_map, "update_gain", &p_update_gain_, 0.0);
            setParam<bool>(param_map, "update_sections_separate",
                           &p_update_sections_separate_, false);

            // setup parent
            YawPlanningEvaluator::setupFromParamMap(param_map);
            p_select_by_value_ = false; // Cannot select by value since views related
        }

        bool ContinuousYawPlanningEvaluator::computeGain(TrajectorySegment *traj_in) {
            // compute gain for all segments as in base class
            YawPlanningEvaluator::computeGain(traj_in);
            // find best stored yaw and apply it
            setBestYaw(traj_in);
        }

        bool ContinuousYawPlanningEvaluator::updateSegment(TrajectorySegment *segment) {
            if (segment->parent && segment->info) {
                double dist = (planner_.getCurrentPosition() - segment->trajectory.back().position_W).norm();
                if (p_update_range_ == 0.0 || p_update_range_ > dist) {
                    bool update_all =
                            (~p_update_sections_separate_) && (segment->gain > p_update_gain_);
                    YawPlanningInfo *info = reinterpret_cast<YawPlanningInfo *>(segment->info.get());
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
            return following_evaluator_->updateSegment(segment);
        }


        void ContinuousYawPlanningEvaluator::setBestYaw(TrajectorySegment *segment) {
            // compute gain from section overlap
            YawPlanningInfo *info =
                    reinterpret_cast<YawPlanningInfo *>(segment->info.get());
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
                best_yaw = defaults::angleScaled(
                        info->orientations[max_index].trajectory.back().getYaw() +
                        (double) p_n_sections_fov_ / p_n_directions_ * M_PI);
            } else {
                // indifferent, use direction of travel
                Eigen::Vector3d direction = segment->trajectory.back().position_W -
                                            segment->trajectory.front().position_W;
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

        double ContinuousYawPlanningEvaluator::sampleYaw(double original_yaw,
                                                         int sample_number) {
            // Uniform sampling
            return defaults::angleScaled(original_yaw + (double) sample_number * 2.0 *
                                                        M_PI / p_n_directions_);
        }

        void ContinuousYawPlanningEvaluator::setTrajectoryYaw(
                TrajectorySegment *segment, double start_yaw, double target_yaw) {
            // just set the yaw of the entire trajectory to the sampled value
            for (int i = 0; i < segment->trajectory.size(); ++i) {
                segment->trajectory[i].setFromYaw(target_yaw);
            }
        }

        void ContinuousYawPlanningEvaluator::visualizeTrajectoryValue(
                VisualizationMarkers *markers, const TrajectorySegment &trajectory) {
            if (!trajectory.info) {
                return;
            }
            YawPlanningInfo *info =
                    reinterpret_cast<YawPlanningInfo *>(trajectory.info.get());
            // Simple version: visualize facing of endpoints of trajectories, colored with
            // value (highest green to lowest red)
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
                VisualizationMarker marker;

                marker.position = info->orientations[i].trajectory.back().position_W;
                marker.orientation = info->orientations[i].trajectory.back().orientation_W_B;
                marker.type = VisualizationMarker::ARROW;
                marker.scale.x() = 0.6;
                marker.scale.y() = 0.07;
                marker.scale.z() = 0.07;
                marker.action = VisualizationMarker::ADD;

                // Color according to relative value (blue when indifferent, grey for values
                // below update range)
                if (info->orientations[i].gain <= p_update_sections_separate_) {
                    marker.color.r = 0.5;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
                } else if (max_value != min_value) {
                    double frac =
                            (info->orientations[i].gain - min_value) / (max_value - min_value);
                    if (p_select_by_value_) {
                        frac =
                                (info->orientations[i].value - min_value) / (max_value - min_value);
                    }
                    marker.color.r = std::min((0.5 - frac) * 2.0 + 1.0, 1.0);
                    marker.color.g = std::min((frac - 0.5) * 2.0 + 1.0, 1.0);
                    marker.color.b = 0.0;
                } else {
                    marker.color.r = 0.3;
                    marker.color.g = 0.3;
                    marker.color.b = 1.0;
                }
                marker.color.a = 0.4;
                markers->addMarker(marker);
            }

            // Followup
            if (p_visualize_followup_) {
                for (int i = 0; i < p_n_sections_fov_; ++i) {
                    following_evaluator_->visualizeTrajectoryValue(markers,
                                                                   info->orientations[(info->active_orientation + i) %
                                                                                      p_n_directions_]);
                }
            }
        }

    } // namespace trajectory_evaluator
} // namespace active_3d_planning
