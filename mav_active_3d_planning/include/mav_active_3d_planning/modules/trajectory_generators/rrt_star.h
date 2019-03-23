#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RRT_STAR_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RRT_STAR_H

#include "mav_active_3d_planning/modules/trajectory_generators/rrt.h"

namespace mav_active_3d_planning {
    class ModuleFactory;

    namespace trajectory_generators {

        // Extension of the RRT class. Use with its trajectory evaluator adapter below to find the gain, cost and value
        // for new segments and tries rewiring the best segments together. RRTStar requires trajectory evaluation based
        // only on the last point, since it evaluates and connects points (and not trajectories).
        class RRTStar : public RRT {
        public:
            // override virtual functions
            virtual bool expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment *> *new_segments);

            bool rewireRoot(TrajectorySegment *root, int next_segment);

        protected:
            friend ModuleFactory;

            // factory access
            RRTStar() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            bool checkParamsValid(std::string *error_message);

            // parameters
            bool p_rewire_root_;        // if true try rewiring the children of the current segment (to keep them alive)
            double p_rewire_range_;     // distance [m] within which we search for alternative segments

            // kdtree
            std::unique_ptr <KDTree> kdtree_;
            TreeData tree_data_;

            // variables


            // methods
            bool findBestParentTrajectory(TrajectorySegment *segment, const std::vector <TrajectorySegment*> &candidates);
//            virtual bool sample_goal(Eigen::Vector3d *goal_pos);
//
//            virtual bool connect_poses(const mav_msgs::EigenTrajectoryPoint &start,
//                                       const mav_msgs::EigenTrajectoryPoint &goal,
//                                       mav_msgs::EigenTrajectoryPointVector *result);
        };

    } // namespace trajectory_generators

    namespace trajectory_evaluators {

        // RRTStar Evaluator adapter, its following evaluator is used for all regular evaluation duties.
        class RRTStarEvaluatorAdapter : public TrajectoryEvaluator {
        public:

            // Override virtual functions
            bool computeGain(TrajectorySegment *traj_in);

            bool computeCost(TrajectorySegment *traj_in);

            bool computeValue(TrajectorySegment *traj_in);

            int selectNextBest(const TrajectorySegment &traj_in);

            bool updateSegments(TrajectorySegment *root);

            void visualizeTrajectoryValue(visualization_msgs::MarkerArray *msg, const TrajectorySegment &trajectory);

        protected:
            friend ModuleFactory;

            // factory access
            RRTStarEvaluatorAdapter() {}

            void setupFromParamMap(Module::ParamMap *param_map);

            // members
            std::unique_ptr <TrajectoryEvaluator> following_evaluator_;
        };

    } //namespace trajectory_evaluators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RRT_STAR_H
