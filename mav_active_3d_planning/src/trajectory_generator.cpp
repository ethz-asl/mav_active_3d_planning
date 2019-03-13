#include "mav_active_3d_planning/trajectory_generator.h"
#include "mav_active_3d_planning/module_factory.h"

#include <ros/param.h>

namespace mav_active_3d_planning {

//    TrajectoryGenerator::TrajectoryGenerator(bool collision_optimistic, double collision_radius,
//                                             std::shared_ptr <voxblox::EsdfServer> voxblox_ptr,
//                                             defaults::BoundingVolume bounding_volume)
//            : p_collision_optimistic(collision_optimistic),
//              p_collision_radius(collision_radius),
//              voxblox_ptr_(voxblox_ptr),
//              bounding_volume_(bounding_volume),
//              p_namespace_(param_ns) {
//
//        //Setup the voxblox server collision for visualization
//        voxblox_ptr_->setTraversabilityRadius(static_cast<float>(p_collision_radius_));
//    }

    void TrajectoryGenerator::setupFromParamMap(Module::ParamMap *param_map) {
        setParam<bool>(param_map, "collision_optimistic", &p_collision_optimistic_, false);
        setParam<double>(param_map, "collision_radius", &p_collision_radius_, 0.35);
        std::string ns = (*param_map)["param_namespace"];
        setParam<std::string>(param_map, "segment_selector_args", &p_selector_args_, ns + "/segment_selector");
        setParam<std::string>(param_map, "generator_updater_args", &p_updater_args_, ns + "/generator_updater");
        std::string temp_args;
        setParam<std::string>(param_map, "bounding_volume_args", &temp_args, ns + "/bounding_volume");
        bounding_volume_ = defaults::BoundingVolume(temp_args);
        setParam<std::string>(param_map, "system_constraints_args", &temp_args, ns + "/system_constraints");
        system_constraints_ = defaults::SystemConstraints(temp_args);
    }

    bool TrajectoryGenerator::checkTraversable(const Eigen::Vector3d &position) {
        if (!bounding_volume_.contains(position)) {
            return false;
        }
        double distance = 0.0;
        if (voxblox_ptr_->getEsdfMapPtr()->getDistanceAtPosition(position, &distance)) {
            return (distance > p_collision_radius_);
        }
        return p_collision_optimistic_;
    }

    bool TrajectoryGenerator::selectSegment(TrajectorySegment **result, TrajectorySegment *root) {
        // If not implemented use a (default) module
        if (!segment_selector_) {
            segment_selector_ = ModuleFactory::Instance()->createSegmentSelector(p_selector_args_, verbose_modules_);
        }
        return segment_selector_->selectSegment(result, root);
    }

    bool TrajectoryGenerator::updateSegments(TrajectorySegment *root) {
        // If not implemented use a (default) module
        if (!generator_updater_) {
            generator_updater_ = ModuleFactory::Instance()->createGeneratorUpdater(p_updater_args_, this, verbose_modules_);
        }
        return generator_updater_->updateSegments(root);
    }

    void TrajectoryGenerator::setVoxbloxPtr(const std::shared_ptr <voxblox::EsdfServer> &voxblox_ptr) {
        voxblox_ptr_ = voxblox_ptr;
        voxblox_ptr_->setTraversabilityRadius(static_cast<float>(p_collision_radius_));
    }

    void GeneratorUpdater::setParent(TrajectoryGenerator *parent) {
        parent_ = parent;
    }

}  // namespace mav_active_3d_planning
