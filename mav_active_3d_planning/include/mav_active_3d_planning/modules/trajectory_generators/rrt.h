#ifndef MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RRT_H
#define MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RRT_H

#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/libs/nanoflann.hpp"

namespace mav_active_3d_planning {
    namespace trajectory_generators {

        // Random Rapidly Exploring Tree with linear segments, similar to the nbvp planner
        class RRT : public TrajectoryGenerator {
        public:
            struct TreeData {
                // data
                std::vector<Eigen::Vector3d>  points;
                std::vector<TrajectorySegment*> data;

                // Clear up everything
                void clear();

                // push back a segment
                void addSegment(TrajectorySegment* to_add);

                // nanoflann functionality
                inline std::size_t kdtree_get_point_count() const { return points.size(); }

                inline double kdtree_get_pt(const size_t idx, const size_t dim) const
                {
                    if (dim == 0) return points[idx].x();
                    else if (dim == 1) return points[idx].y();
                    else return points[idx].z();
                }

                template <class BBOX>
                bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
            };

            typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, TreeData>,
                    TreeData, 3> KDTree;

            // override virtual functions
            bool selectSegment(TrajectorySegment **result, TrajectorySegment *root);
            bool expandSegment(TrajectorySegment *target, std::vector<TrajectorySegment*> *new_segments);

        protected:
            friend ModuleFactory;

            // factory access
            RRT() {}
            void setupFromParamMap(Module::ParamMap *param_map);
            static ModuleFactory::Registration<RRT> registration;

            // parameters
            double p_velocity_;         // m/s
            double p_sampling_rate_;    // Hz
            double p_extension_range_;  // m (set 0.0 to ignore)
            bool p_use_spheric_sampling_;
            int p_maximum_tries_;       // sampling tries, 0 for inf

            // kdtree
            std::unique_ptr<KDTree> kdtree_;
            TreeData tree_data_;

            // variables
            TrajectorySegment *previous_root_;
            Eigen::Vector3d goal_pos_;

            // methods
            bool sample_spheric(Eigen::Vector3d* goal_pos);
            bool sample_box(Eigen::Vector3d* goal_pos);
        };

    } // namespace trajectory_generators
}  // namespace mav_active_3d_planning
#endif // MAV_ACTIVE_3D_PLANNING_TRAJECTORY_GENERATORS_RRT_H
