#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/planner_node.h"

namespace mav_active_3d_planning {

    void TrajectoryGenerator::setupFromParamMap(Module::ParamMap *param_map) {
        setParam<bool>(param_map, "collision_optimistic", &p_collision_optimistic_, false);
        setParam<double>(param_map, "collision_radius", &p_collision_radius_, 0.35);
        setParam<double>(param_map, "clearing_radius", &p_clearing_radius_, 0.0);
        std::string ns = (*param_map)["param_namespace"];
        setParam<std::string>(param_map, "segment_selector_args", &p_selector_args_, ns + "/segment_selector");
        setParam<std::string>(param_map, "generator_updater_args", &p_updater_args_, ns + "/generator_updater");
        std::string temp_args;
        setParam<std::string>(param_map, "bounding_volume_args", &temp_args, ns + "/bounding_volume");
        bounding_volume_ = ModuleFactory::Instance()->createModule<defaults::BoundingVolume>(temp_args,
                                                                                             verbose_modules_);
        setParam<std::string>(param_map, "system_constraints_args", &temp_args, ns + "/system_constraints");
        system_constraints_ = ModuleFactory::Instance()->createModule<defaults::SystemConstraints>(temp_args,
                                                                                                   verbose_modules_);
        // Get the voxblox server
        planner_node_ = dynamic_cast<PlannerNode *>(ModuleFactory::Instance()->readLinkableModule("PlannerNode"));
        voxblox_ptr_.reset(planner_node_->voxblox_server_.get());
    }

    bool TrajectoryGenerator::checkTraversable(const Eigen::Vector3d &position) {
        if (!bounding_volume_->contains(position)) {
            return false;
        }
        double distance = 0.0;
        if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
            // This means the voxel is observed
            return (distance > p_collision_radius_);
        }
        if (p_clearing_radius_ > 0.0){
            if ((planner_node_->getCurrentPosition()-position).norm() < p_clearing_radius_){
                return true;
            }
        }
        return p_collision_optimistic_;
    }

    bool TrajectoryGenerator::selectSegment(TrajectorySegment **result, TrajectorySegment *root) {
        // If not implemented use a (default) module
        if (!segment_selector_) {
            segment_selector_ = ModuleFactory::Instance()->createModule<SegmentSelector>(p_selector_args_,
                                                                                         verbose_modules_);
        }
        return segment_selector_->selectSegment(result, root);
    }

    bool TrajectoryGenerator::updateSegments(TrajectorySegment *root) {
        // If not implemented use a (default) module
        if (!generator_updater_) {
            generator_updater_ = ModuleFactory::Instance()->createModule<GeneratorUpdater>(p_updater_args_,
                                                                                           verbose_modules_);
        }
        return generator_updater_->updateSegments(root);
    }

}  // namespace mav_active_3d_planning
