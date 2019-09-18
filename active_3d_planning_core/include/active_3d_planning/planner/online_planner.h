#ifndef ACTIVE_3D_PLANNING_CORE_PLANNER_ONLINE_PLANNER_H
#define ACTIVE_3D_PLANNING_CORE_PLANNER_ONLINE_PLANNER_H

#include "active_3d_planning/data/trajectory_segment.h"
#include "active_3d_planning/data/visualization_markers.h"
#include "active_3d_planning/module/back_tracker.h"
#include "active_3d_planning/module/module_factory.h"
#include "active_3d_planning/module/trajectory_evaluator.h"
#include "active_3d_planning/module/trajectory_generator.h"
#include "active_3d_planning/planner/planner_I.h"
#include "active_3d_planning/module/module.h"
#include "active_3d_planning/tools/defaults.h"

#include <voxblox_ros/esdf_server.h>

#include <ctime>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace active_3d_planning {

        class OnlinePlanner : public PlannerI, ModuleBase {
        public:
            OnlinePlanner(ModuleFactory* factory, std::string module_args);

            virtual ~OnlinePlanner() = default;

            // Start/Run the planning loop
            void planningLoop();

            // Accessors
            const Eigen::Vector3d &getCurrentPosition() const override {
                return current_position_;
            }
            const Eigen::Quaterniond &getCurrentOrientation() const override {
                return current_orientation_;
            }

            virtual BackTracker &getBackTracker() override {
                return *back_tracker_;
            }
            virtual TrajectoryGenerator &getTrajectoryGenerator() override {
                return *trajectory_generator_;
            }
            virtual TrajectoryEvaluator &getTrajectoryEvaluator() override {
                return *trajectory_evaluator_;
            }

            virtual void publishVisualization(const VisualizationMarkers& markers) = 0;

            virtual ModuleFactory &getFactory() override {
                return *factory_;
            }

            // maybe want to get rid of voxblox at some point
            virtual voxblox::EsdfMap &getMap() override {
                return *(voxblox_server_->getEsdfMapPtr());
            }
            virtual voxblox::TsdfMap &getTsdfMap() override {
                return *(voxblox_server_->getTsdfMapPtr());
            }

        protected:
            // factory
            ModuleFactory* factory_;

            // members
            std::shared_ptr<voxblox::EsdfServer> voxblox_server_;
            std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
            std::unique_ptr<TrajectoryEvaluator> trajectory_evaluator_;
            std::unique_ptr<BackTracker> back_tracker_;
            std::unique_ptr<TrajectorySegment> current_segment_;    // root node of full trajectory tree

            // variables
            bool running_;                    // whether to run the main loop
            Eigen::Vector3d current_position_; // Current pose of the robot
            Eigen::Quaterniond current_orientation_;
            bool target_reached_; // whether the goal point was reached, update based on odom input
            int new_segments_; // keep track of min/max tries and segments
            int new_segment_tries_;
            bool min_new_value_reached_;
            std::string logfile_;

            // Info+performance bookkeeping
            std::clock_t info_timing_;    // Simulated/On-Board time for verbose and perf [s]
            int info_count_;              // num trajectories counter for verbose
            int info_killed_next_;        // number of segments killed during root change
            int info_killed_update_;      // number of segments killed during updating
            std::ofstream perf_log_file_; // performance file
            std::vector<double>  perf_log_data_; // select, expand, gain, cost, value [cpu seconds]
            std::clock_t perf_cpu_timer_; // total time counter

            // params
            bool p_verbose_;
            bool p_visualize_; // Publish visualization of completed path, current gain,
            // new candidates, ...
            bool p_log_performance_;     // Whether to write a performance log file
            int p_max_new_segments_; // After this count is reached no more segments are
            // expanded (0 to ignore)
            int p_min_new_segments_; // Until this count is reached the next segment is
            // not executed (0 to ignore)
            int p_min_new_tries_;    // Until no. expansion calls the next segment is not
            // executed (0 to ignore)
            int p_max_new_tries_; // After no. expansion calls the next segment is forced
            // (0 to ignore)
            double p_min_new_value_; // Until this value is found in the tree, expansion
            // continues (0 to ignore)
            int p_expand_batch_;     // run multiple segment expansions before rechecking
            // conditions
            bool p_visualize_gain_; // true: add a colored sphere according to the gain to
            // every segment
            bool p_highlight_executed_trajectory_;  // true: print executed trajectory in bold red

            // methods
            virtual void initializePlanning();

            virtual void loopIteration();

            virtual void requestNextTrajectory();

            virtual void expandTrajectories();

            virtual void requestMovement(const TrajectorySegment &req) = 0;

            virtual void setupFromParamMap(ParamMap *param_map);

            // visualization
            void publishTrajectoryVisualization(
                    const std::vector<TrajectorySegment *> &trajectories);

            void publishCompletedTrajectoryVisualization(
                    const TrajectorySegment &trajectories);

            void publishEvalVisualization(const TrajectorySegment &trajectory);

            // factory access
            void setupFromParamMap(Module::ParamMap *param_map) {}

            // helper functions
            bool checkMinNewValue(const std::unique_ptr<TrajectorySegment>
                                  &segment); // check whether min value is found
        };
} // namespace active_3d_planning

#endif //ACTIVE_3D_PLANNING_CORE_PLANNER_ONLINE_PLANNER_H
