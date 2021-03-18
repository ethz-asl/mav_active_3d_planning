#include "active_3d_planning_core/module/trajectory_generator/generator_updater/simple_rrt_star_collision_updater.h"

#include <algorithm>
#include <string>
#include <vector>

#include "active_3d_planning_core/module/module_factory.h"
#include "active_3d_planning_core/planner/planner_I.h"

namespace active_3d_planning {
namespace generator_updater {

ModuleFactoryRegistry::Registration<SimpleRRTStarCollisionUpdater>
    SimpleRRTStarCollisionUpdater::registration(
        "SimpleRRTStarCollisionUpdater");

SimpleRRTStarCollisionUpdater::SimpleRRTStarCollisionUpdater(PlannerI& planner)
    : GeneratorUpdater(planner) {}

void SimpleRRTStarCollisionUpdater::setupFromParamMap(
    Module::ParamMap* param_map) {
  generator_ = static_cast<trajectory_generator::RRTStar*>(
      planner_.getFactory().readLinkableModule("RRTStarGenerator"));

  // Create following updater
  std::string args;  // default args extends the parent namespace
  std::string param_ns = (*param_map)["param_namespace"];
  setParam<std::string>(param_map, "following_updater_args", &args,
                        param_ns + "/following_updater");
  following_updater_ = planner_.getFactory().createModule<GeneratorUpdater>(
      args, planner_, verbose_modules_);
}

bool SimpleRRTStarCollisionUpdater::updateSegment(TrajectorySegment* segment) {
  TrajectorySegment* root;
  if (!(segment->parent->parent)) {
    root = segment->parent;
    if (root != previous_root_) {
      previous_root_ = root;
      // only execute the full update step once
      // find all colliding segments
      std::vector<TrajectorySegment*> to_rewire;
      std::vector<TrajectorySegment*> safe_segments;
      safe_segments.push_back(root);
      checkSingle(root, &to_rewire, &safe_segments);
      if (to_rewire.size() > 0) {
        generator_->resetTree(root);
        std::vector<TrajectorySegment*> candidate_parents;
        std::vector<TrajectorySegment*> safe_parents;

        // try to rewire all segments once
        int index = 0;
        while (index < to_rewire.size()) {
          candidate_parents.clear();
          if (!generator_->findNearbyCandidates(
                  to_rewire[index]->trajectory.back().position_W,
                  &candidate_parents)) {
            to_rewire[index]->getChildren(&to_rewire);
            ++index;
            continue;
          }
          safe_parents.clear();
          for (int j = 0; j < candidate_parents.size(); ++j) {
            if (std::find(safe_segments.begin(), safe_segments.end(),
                          candidate_parents[j]) != safe_segments.end()) {
              safe_parents.push_back(candidate_parents[j]);
            }
          }
          if (safe_parents.empty()) {
            to_rewire[index]->getChildren(&to_rewire);
            ++index;
            continue;
          }
          generator_->rewireToBestParent(to_rewire[index], safe_parents, true);
          safe_segments.push_back(to_rewire[index]);
          to_rewire.erase(to_rewire.begin() + index);
        }

        // kill all segments that could not be rewired
        while (to_rewire.size() > 0) {
          TrajectorySegment* current = to_rewire[0];
          std::vector<TrajectorySegment*> killed;
          current->getTree(&killed);
          to_rewire.erase(remove_if(to_rewire.begin(), to_rewire.end(),
                                    [&](TrajectorySegment* x) {
                                      return find(killed.begin(), killed.end(),
                                                  x) != killed.end();
                                    }),
                          to_rewire.end());
          for (int j = 0; j < current->parent->children.size(); ++j) {
            if (current->parent->children[j].get() == current) {
              current->parent->children.erase(
                  current->parent->children.begin() + j);
              break;
            }
          }
        }
      }
    }
  }
  return following_updater_->updateSegment(segment);
}

bool SimpleRRTStarCollisionUpdater::isCollided(
    const EigenTrajectoryPointVector& trajectory) {
  for (int i = 0; i < trajectory.size(); ++i) {
    if (!(generator_->checkTraversable(trajectory[i].position_W))) {
      return true;
    }
  }
  return false;
}

void SimpleRRTStarCollisionUpdater::checkSingle(
    TrajectorySegment* segment, std::vector<TrajectorySegment*>* to_rewire,
    std::vector<TrajectorySegment*>* safe_segments) {
  // recursively find all colliding segments, excluding the root
  for (int i = 0; i < segment->children.size(); ++i) {
    if (isCollided(segment->children[i]->trajectory)) {
      to_rewire->push_back(segment->children[i].get());
    } else {
      safe_segments->push_back(segment->children[i].get());
      checkSingle(segment->children[i].get(), to_rewire, safe_segments);
    }
  }
}

}  // namespace generator_updater
}  // namespace active_3d_planning
