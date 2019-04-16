#include "mav_active_3d_planning/modules/trajectory_generators/generator_updaters/default_generator_updaters.h"

#include <vector>
#include <algorithm>
#include <memory>

namespace mav_active_3d_planning {
    namespace generator_updaters {

        ModuleFactory::Registration<GeneratorUpdateNothing> GeneratorUpdateNothing::registration("GeneratorUpdateNothing");

        // GeneratorResetTree
        ModuleFactory::Registration<GeneratorResetTree> GeneratorResetTree::registration("GeneratorResetTree");

        bool GeneratorResetTree::updateSegments(TrajectorySegment *root) {
            root->children.clear();
            return true;
        }

        // RecheckCollision
        ModuleFactory::Registration<RecheckCollision> RecheckCollision::registration("RecheckCollision");

        void RecheckCollision::setupFromParamMap(Module::ParamMap *param_map) {
            planner_node_ = dynamic_cast<PlannerNode *>(ModuleFactory::Instance()->readLinkableModule("PlannerNode"));
        }

        bool RecheckCollision::updateSegments(TrajectorySegment *root) {
            checkSingle(root);
            return true;
        }

        bool RecheckCollision::isCollided(const mav_msgs::EigenTrajectoryPointVector &trajectory) {
            for (int i = 0; i < trajectory.size(); ++i) {
                if (!(planner_node_->trajectory_generator_->checkTraversable(trajectory[i].position_W))) {
                    return true;
                }
            }
            return false;
        }

        void RecheckCollision::checkSingle(TrajectorySegment *segment) {
            // Recursive removal
            int j = 0;
            for (int i = 0; i < segment->children.size(); ++i) {
                if (isCollided(segment->children[j]->trajectory)) {
                    segment->children.erase(segment->children.begin() + j);
                } else {
                    j++;
                }
            }
            // remaining children
            for (int i = 0; i < segment->children.size(); ++i) {
                checkSingle(segment->children[i].get());
            }
        }

    } // namespace generator_updaters
} // namepsace mav_active_3d_planning
