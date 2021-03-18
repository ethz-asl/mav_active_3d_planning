#ifndef ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_SEGMENT_H_
#define ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_SEGMENT_H_

#include <memory>
#include <vector>

#include <active_3d_planning_core/data/trajectory.h>

namespace active_3d_planning {

// Base struct that contains trajectory evaluator information associated with
// the segment.
struct TrajectoryInfo {
  virtual ~TrajectoryInfo() = default;
};

// Struct to store trajectory tree data
struct TrajectorySegment {
  TrajectorySegment();

  // All trajectory points
  EigenTrajectoryPointVector trajectory;

  // Associated values
  double gain;
  double cost;
  double value;

  // flags
  bool tg_visited;  // trajectory generator visited during expansion site
  // selection

  // pointer to parent trajectory, nullptr for currently active segment (root)
  TrajectorySegment* parent;

  // Pointers to successive trajectory nodes, all nodes are owned by the parent
  std::vector<std::unique_ptr<TrajectorySegment>> children;

  // Information to be carried with this segment, e.g. virtual voxels
  std::unique_ptr<TrajectoryInfo> info;

  // compare function for sorting etc
  static bool compare(TrajectorySegment a, TrajectorySegment b);

  static bool comparePtr(TrajectorySegment* a, TrajectorySegment* b);

  // Safely create a child node and return a pointer to it
  TrajectorySegment* spawnChild();

  // The following utility functions assume a tree structure (no loops)
  // Add pointers to all immediate children to the result vector
  void getChildren(std::vector<TrajectorySegment*>* result);

  // Recursively add pointers to all leaf nodes (have no children) to the result
  // vector
  void getLeaves(std::vector<TrajectorySegment*>* result);

  // Recursively add pointers to all subsequent nodes to the result vector
  void getTree(std::vector<TrajectorySegment*>* result);

  // Recursively add pointers to all subsequent nodes to the result vector up to
  // given depth
  void getTree(std::vector<TrajectorySegment*>* result, int maxdepth);

  // Create a shallow copy of the segment (Includes everything except the unique
  // pointers children and info, which will not be setup!)
  TrajectorySegment shallowCopy();
};

}  // namespace active_3d_planning
#endif  // ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_SEGMENT_H_
