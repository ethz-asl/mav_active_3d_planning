#include "mav_active_3d_planning/trajectory_generator.h"

#include <ros/param.h>

#include <vector>
#include <algorithm>
#include <random>
#include <memory>

namespace mav_active_3d_planning {
    namespace generator_updaters {

        // Discard all segments and start from scratch
        class Clear : public GeneratorUpdater {
        public:
            Clear() {}

            bool updateSegments(TrajectorySegment &root) {
                root.children.clear();
                return true;
            }
        };

        // Don't perform specific update operations
        class Void : public GeneratorUpdater {
        public:
            Void() {}

            bool updateSegments(TrajectorySegment &root) {
                return true;
            }
        };

        // Recursively check wether the trajectories are still collision free
        class CheckCollision : public GeneratorUpdater {
        public:
            CheckCollision(TrajectoryGenerator* parent) : GeneratorUpdater(parent) {}

            bool updateSegments(TrajectorySegment &root) {
                checkSingle(root);
                return true;
            }

            bool isCollided(mav_msgs::EigenTrajectoryPointVector &trajectory) {
                for (int i = 0; i < trajectory.size(); ++i) {
                    if (!parent_->checkTraversable(trajectory[i].position_W)){
                        return true;
                    }
                }
                return false;
            }

            void checkSingle(TrajectorySegment &segment){
                // Recursive removal
                int j = 0;
                for (int i = 0; i < segment.children.size(); ++i) {
                    if (isCollided(segment.children[j]->trajectory)){
                        segment.children.erase(segment.children.begin() + j);
                    } else {
                        j++;
                    }
                }
                // remaining children
                for (int i = 0; i < segment.children.size(); ++i) {
                    checkSingle(*(segment.children[i]));
                }
            }
        };

    } // namespace generator_updaters
} // namepsace mav_active_3d_planning
