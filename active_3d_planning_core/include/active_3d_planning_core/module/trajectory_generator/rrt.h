#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_RRT_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_RRT_H_

#include <memory>
#include <string>
#include <vector>

#include "active_3d_planning_core/libs/nanoflann.hpp"
#include "active_3d_planning_core/module/trajectory_generator.h"

namespace active_3d_planning {
namespace trajectory_generator {

// Random Rapidly Exploring Tree with linear segments, similar to the nbvp
// planner
class RRT : public TrajectoryGenerator {
 public:
  struct TreeData {
    // data
    std::vector<Eigen::Vector3d> points;
    std::vector<TrajectorySegment*> data;

    // Clear up everything
    void clear();

    // push back a segment
    void addSegment(TrajectorySegment* to_add);

    // nanoflann functionality
    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
      if (dim == 0)
        return points[idx].x();
      else if (dim == 1)
        return points[idx].y();
      else
        return points[idx].z();
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const {
      return false;
    }
  };

  typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
      nanoflann::L2_Simple_Adaptor<double, TreeData>, TreeData, 3>
      KDTree;

  explicit RRT(PlannerI& planner);  // NOLINT

  // override virtual functions
  bool selectSegment(TrajectorySegment** result,
                     TrajectorySegment* root) override;

  bool expandSegment(TrajectorySegment* target,
                     std::vector<TrajectorySegment*>* new_segments) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  bool checkParamsValid(std::string* error_message) override;

 protected:
  static ModuleFactoryRegistry::Registration<RRT> registration;

  // parameters
  bool p_crop_segments_;  // if true extend segments as long as possible instead
  // of cancling them
  double p_crop_margin_;  // margin [m] to intraversable points when cropping
  double
      p_crop_min_length_;   // Cropped segments below this length are discarded
  double p_sampling_rate_;  // Hz
  double p_max_extension_range_;   // m (set 0.0 to ignore)
  std::string p_sampling_mode_;    // uniform, spheric, semilocal
  double p_semilocal_radius_max_;  // Only used for semilocal sampling, radius
  // where points are counted
  double p_semilocal_radius_min_;  // Only used for semilocal sampling, min
  // distance of new points
  int p_semilocal_count_;  // Only used for semilocal sampling, min number of
  // points in radius
  bool p_sample_yaw_;    // True: random yaw, false: face direction of travel
  int p_maximum_tries_;  // sampling tries, 0 for inf
  double p_min_path_length_;  // disallow all segments shorter than min length

  // kdtree
  std::unique_ptr<KDTree> kdtree_;
  TreeData tree_data_;

  // variables
  TrajectorySegment* previous_root_;
  Eigen::Vector3d goal_pos_;
  int semilocal_count_;

  // methods
  // find a goal position, return true on success.
  virtual bool sampleGoal(Eigen::Vector3d* goal_pos);

  // try connecting two poses and build a trajectory in between them. Return
  // true on success.
  virtual bool connectPoses(const EigenTrajectoryPoint& start,
                            const EigenTrajectoryPoint& goal,
                            EigenTrajectoryPointVector* result,
                            bool check_collision = true);

  // check max segment length and cropping
  virtual bool adjustGoalPosition(const Eigen::Vector3d& start_pos,
                                  Eigen::Vector3d* goal_pos_);

  bool resetTree(TrajectorySegment* root);
};

}  // namespace trajectory_generator
}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_GENERATOR_RRT_H_
