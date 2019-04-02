#include "mav_active_3d_planning/modules/trajectory_evaluators/evaluator_updaters/yaw_planning_updaters.h"
#include "mav_active_3d_planning/module_factory.h"


namespace mav_active_3d_planning {
    namespace evaluator_updaters {
        using YawPlanningInfo = mav_active_3d_planning::trajectory_evaluators::YawPlanningInfo;

        // YawPlanningUpdateAdapter
        ModuleFactory::Registration <YawPlanningUpdateAdapter> YawPlanningUpdateAdapter::registration(
                "YawPlanningUpdateAdapter");

        bool YawPlanningUpdateAdapter::updateSegments(TrajectorySegment *root) {
            // recursively update all segments, exclude root since this is now the base with no parent
            for (int i = 0; i < root->children.size(); ++i) {
                updateSingle(root->children[i].get());
            }
            return true;
        }

        void YawPlanningUpdateAdapter::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "dynamic_trajectories", &p_dynamic_trajectories_, false);
            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createModule<EvaluatorUpdater>(args, verbose_modules_,
                                                                                           parent_);
        }

        void YawPlanningUpdateAdapter::updateSingle(TrajectorySegment *segment) {
            if (segment->info) {
                YawPlanningInfo *info = dynamic_cast<YawPlanningInfo *>(segment->info.get());
                if (p_dynamic_trajectories_) {
                    // Update the (possibly changed) trajectory
                    info->orientations[info->active_orientation].trajectory = segment->trajectory;
                    info->orientations[info->active_orientation].parent = segment->parent;
                }
                // Update the stored active orientation
                info->orientations[info->active_orientation].parent = segment->parent;
                following_updater_->updateSegments(&(info->orientations[info->active_orientation]));
                segment->gain = info->orientations[info->active_orientation].gain;
                segment->cost = info->orientations[info->active_orientation].cost;
                segment->value = info->orientations[info->active_orientation].value;
            }

            // Propagate through trajectory tree
            for (int i = 0; i < segment->children.size(); ++i) {
                updateSingle(segment->children[i].get());
            }
        }

        // YawPlanningUpdater
        ModuleFactory::Registration <YawPlanningUpdater> YawPlanningUpdater::registration("YawPlanningUpdater");

        bool YawPlanningUpdater::updateSegments(TrajectorySegment *root) {
            // recursively update all segments, exclude root since this is now the base with no parent
            for (int i = 0; i < root->children.size(); ++i) {
                updateSingle(root->children[i].get());
            }
            return true;
        }

        void YawPlanningUpdater::setupFromParamMap(Module::ParamMap *param_map) {
            setParam<bool>(param_map, "select_by_value", &p_select_by_value_, false);
            setParam<bool>(param_map, "dynamic_trajectories", &p_dynamic_trajectories_, false);
            evaluator_ = dynamic_cast<YawPlanningEvaluator *>(ModuleFactory::Instance()->readLinkableModule(
                    "YawPlanningEvaluator"));

            // Create Following updater (default does nothing)
            std::string args;   // default args extends the parent namespace
            std::string param_ns = (*param_map)["param_namespace"];
            setParam<std::string>(param_map, "following_updater_args", &args, param_ns + "/following_updater");
            following_updater_ = ModuleFactory::Instance()->createModule<EvaluatorUpdater>(args, verbose_modules_,
                                                                                           parent_);
        }

        void YawPlanningUpdater::updateSingle(TrajectorySegment *segment) {
            if (segment->info) {
                YawPlanningInfo *info = dynamic_cast<YawPlanningInfo *>(segment->info.get());

                // If requested, check whether the trajectory changed
                if (p_dynamic_trajectories_) {
                    if (info->orientations[info->active_orientation].cost != segment->cost ||
                        info->orientations[info->active_orientation].value != segment->value ||
                        info->orientations[info->active_orientation].parent != segment->parent) {
                        // Trajectory changed, recompute all orientations
                        for (int j = 0; j < info->orientations.size(); ++j) {
                            info->orientations[j].parent = segment->parent;
                            double yaw = info->orientations[j].trajectory.back().getYaw();
                            info->orientations[j].trajectory = segment->trajectory;
                            evaluator_->setTrajectoryYaw(&(info->orientations[j]), segment->trajectory.front().getYaw(), yaw);
                        }
                    }
                }

                // Update all orientations with the following updater and select the best one as the main trajectory
                // Initialization step
                following_updater_->updateSegments(&(info->orientations[0]));
                info->active_orientation = 0;
                double best_value = info->orientations[0].gain;
                if (p_select_by_value_) {
                    best_value = info->orientations[0].value;
                }

                // Update and compare all other orientations
                for (int i = 1; i < info->orientations.size(); ++i) {
                    following_updater_->updateSegments(&(info->orientations[i]));
                    if (p_select_by_value_) {
                        if (info->orientations[i].value > best_value) {
                            info->active_orientation = i;
                            best_value = info->orientations[i].value;
                        }
                    } else {
                        if (info->orientations[i].gain > best_value) {
                            info->active_orientation = i;
                            best_value = info->orientations[i].gain;
                        }
                    }
                }

                // Apply best rotation to the segment
                segment->trajectory = info->orientations[info->active_orientation].trajectory;
                segment->gain = info->orientations[info->active_orientation].gain;
                if (p_select_by_value_) {
                    segment->cost = info->orientations[info->active_orientation].cost;
                    segment->value = info->orientations[info->active_orientation].value;
                }
            }

            // Propagate through trajectory tree
            for (int i = 0; i < segment->children.size(); ++i) {
                updateSingle(segment->children[i].get());
            }
        }

    } // namespace evaluator_updaters
} // namepsace mav_active_3d_planning
