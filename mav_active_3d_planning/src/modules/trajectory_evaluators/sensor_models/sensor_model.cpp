#include "mav_active_3d_planning/modules/trajectory_evaluators/sensor_models/sensor_model.h"

namespace mav_active_3d_planning {

        // SensorModel
        void SensorModel::setVoxbloxPtr(const std::shared_ptr <voxblox::EsdfServer> &voxblox_ptr) {
            voxblox_ptr_ = voxblox_ptr;

            // Cache constants from voxblox
            c_voxel_size_ = voxblox_ptr_->getEsdfMapPtr()->voxel_size();
            c_block_size_ = voxblox_ptr_->getEsdfMapPtr()->block_size();
            voxblox_ptr_->getEsdfMapPtr();
        }

        bool SensorModel::getVoxelCenter(Eigen::Vector3d *point) {
            // Move the input point to the voxel center of its voxel
            voxblox::BlockIndex block_id = voxblox_ptr_->getEsdfMapPtr()->getEsdfLayerPtr()->
                    computeBlockIndexFromCoordinates(point->cast<voxblox::FloatingPoint>());
            Eigen::Vector3d center_point =
                    voxblox::getOriginPointFromGridIndex(block_id, c_block_size_).cast<double>();
            voxblox::VoxelIndex voxel_id = voxblox::getGridIndexFromPoint<voxblox::VoxelIndex>(
                    (*point - center_point).cast<voxblox::FloatingPoint>(), 1.0 / c_voxel_size_);
            center_point += voxblox::getCenterPointFromGridIndex(voxel_id, c_voxel_size_).cast<double>();
            *point = center_point;
            return true;
        }

} // namepsace mav_active_3d_planning