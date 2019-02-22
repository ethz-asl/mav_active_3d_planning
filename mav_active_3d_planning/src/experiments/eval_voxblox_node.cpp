#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <std_srvs/Empty.h>

#include <deque>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <ctime>
#include <stdio.h>
#include <cmath>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>
#include <voxblox_ros/mesh_vis.h>
#include <voxblox_ros/ptcloud_vis.h>

namespace mav_active_3d_planning {

    // Ros-wrapper for c++ voxblox code to evaluates voxblox maps upon request from the eval_plotting_node.
    // Largely based on the voxblox_ros/voxblox_eval.cc code.
    // Pretty ugly and non-general code but just needs to work in this specific case atm...
    class EvaluationNode {
    public:
        EvaluationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

        bool evaluate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        std::string evaluateSingle(std::string map_name);
        void writeLog(std::string text);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::ServiceServer eval_srv_;

        // params
        std::string p_target_dir_;
        bool p_create_meshes_;
        bool p_evaluate_;

        // log file
        std::string log_file_path_;
        std::ofstream log_file_;

        // Ground truth pointcloud
        pcl::PointCloud<pcl::PointXYZRGB> gt_ptcloud_;
    };

    EvaluationNode::EvaluationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
            : nh_(nh),
              nh_private_(nh_private) {
                   eval_srv_ = nh_private_.advertiseService("evaluate", &EvaluationNode::evaluate, this);
    }

    bool EvaluationNode::evaluate(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        // Get target directory
        nh_private_.getParam("target_directory", p_target_dir_);
        ROS_INFO("Starting voxblox evaluation.");
        log_file_path_ = p_target_dir_+ "/data_log.txt";
        std::ifstream read_log(log_file_path_.c_str());
        std::ifstream data_file((p_target_dir_+ "/voxblox_data.csv").c_str());
        if( !(read_log.is_open() && data_file.is_open())){
            ROS_ERROR("Unable to load the data and/or log files.");
            read_log.close();
            data_file.close();
            return false;
        }

        // Get actions to perform
        nh_private_.param("evaluate", p_evaluate_, true);
        nh_private_.param("create_meshes", p_create_meshes_, true);

        // Check not previously evaluated
        std::string line;
        if (p_evaluate_) {
            while (std::getline(read_log, line)) {
                if (line == "[FLAG] Evaluated") {
                    ROS_INFO("This file was already evaluated.");
                    p_evaluate_ = false;
                }
            }
        }
        read_log.close();
        if (!p_evaluate_ && !p_create_meshes_){
            return true;
        }

        // Setup log file
        log_file_.open(log_file_path_.c_str(), std::ios::app);

        // Load the groundtruth pointcloud.
        std::string gt_path;
        nh_private_.getParam("ground_truth_path", gt_path);
        pcl::PLYReader ply_reader;
        if( ply_reader.read(gt_path, gt_ptcloud_) != 0){
            ROS_ERROR("Unable to load ground truth pointcloudfrom '%s'.", gt_path.c_str());
            return false;
        }
        ROS_INFO("Succesfully loaded ground truth pointcloud from '%s'.", gt_path.c_str());

        // Parse and evaluate all lines in the data file
        std::string map_name;
        int n_maps = 0;
        std::ofstream fout;
        if (p_evaluate_) {
            fout.open((p_target_dir_ + "/voxblox_data_temp.csv").c_str(), std::ios::out);
        }
        ROS_INFO("Start processing maps.");
        while (std::getline(data_file, line)){
            line.pop_back();
            map_name = line.substr(0, line.find(","));
            if (map_name == "MapName") {
                map_name = "Header";
            } else if (map_name == "Unit") {
                map_name = "Unit";
            } else {
                // Check map exists
                if (!std::ifstream((p_target_dir_ + "/voxblox_maps/" + map_name + ".vxblx").c_str())) {
                    ROS_INFO("Skipping map '%s' (non-existant)", map_name.c_str());
                    if (p_evaluate_) {
                        fout << line + ",,,,\n";
                    }
                    continue;
                }
                n_maps++;
            }
            ROS_INFO("Processing: %s", map_name.c_str());
            if (p_evaluate_) {
                fout << line + "," + evaluateSingle(map_name) + "\n";
            } else if (p_create_meshes_){
                evaluateSingle(map_name);
            }
        }
        data_file.close();
        if (p_evaluate_) {
            fout.close();
        }
        ROS_INFO("Finished processing maps.");

        // Finish
        if (p_evaluate_) {
            if (std::rename((p_target_dir_ + "/voxblox_data_temp.csv").c_str(),
                            (p_target_dir_ + "/voxblox_data.csv").c_str()) != 0) {
                ROS_ERROR("Could not rename temp csv file!");
            }
            writeLog("Evaluated " + std::to_string(n_maps) +
                     " voxblox maps. Fields: 'MeanError', 'StdDevError', 'UnknownVoxels', 'OutsideTruncation'.");
            log_file_ << "[FLAG] Evaluated\n";
        }
        if (p_create_meshes_) {
            writeLog("Created " + std::to_string(n_maps) + " color and error meshes.");
        }
        log_file_.close();
        ROS_INFO("Voxblox evaluation finished succesfully.");
        return true;
    }

    std::string EvaluationNode::evaluateSingle(std::string map_name) {
        if (map_name == "Header"){
            return "MeanError,StdDevError,UnknownVoxels,OutsideTruncation";
        } else if (map_name == "Unit"){
            return "meters,meters,percent,percent";
        }

        // Setup the voxblox map and interpolater
        std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
        voxblox::Interpolator<voxblox::TsdfVoxel>::Ptr interpolator;
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(p_target_dir_ + "/voxblox_maps/" + map_name + ".vxblx", &tsdf_layer);
        interpolator.reset(new voxblox::Interpolator<voxblox::TsdfVoxel>(tsdf_layer.get()));

        // create color mesh
        voxblox::MeshIntegratorConfig mesh_config;
        std::shared_ptr<voxblox::MeshLayer> mesh_layer;
        std::shared_ptr<voxblox::MeshIntegrator<voxblox::TsdfVoxel>> mesh_integrator;
        constexpr bool only_mesh_updated_blocks = false;
        constexpr bool clear_updated_flag = true;
        if (p_create_meshes_) {
            mesh_layer.reset(new voxblox::MeshLayer(tsdf_layer->block_size()));
            mesh_integrator.reset(new voxblox::MeshIntegrator<voxblox::TsdfVoxel>(
                    mesh_config, tsdf_layer.get(), mesh_layer.get()));
            mesh_integrator->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
            voxblox::outputMeshLayerAsPly(p_target_dir_ + "/meshes/" +map_name + "color.ply", *mesh_layer);

            // Set every voxel to gray for error map
            voxblox::BlockIndexList blocks;
            tsdf_layer->getAllAllocatedBlocks(&blocks);
            const size_t vps = tsdf_layer->voxels_per_side();
            const size_t num_voxels_per_block = vps * vps * vps;
            for (voxblox::BlockIndex& index : blocks) {
                // Iterate over all voxels in said blocks.
                voxblox::Block <voxblox::TsdfVoxel> &block = tsdf_layer->getBlockByIndex(index);
                for (size_t linear_index = 0; linear_index < num_voxels_per_block;
                     ++linear_index) {
                    voxblox::TsdfVoxel &voxel = block.getVoxelByLinearIndex(linear_index);
                    voxel.color = voxblox::Color(128, 128, 128);
                }
            }
        }

        // Go through each point, use trilateral interpolation to figure out the distance at that point.
        // This double-counts -- multiple queries to the same voxel will be counted separately.
        // Coloring: grey -> unknown, green -> 0 error, red -> maximum error (truncation dist=2voxelsize)
        uint64_t total_evaluated_voxels = 0;
        uint64_t unknown_voxels = 0;
        uint64_t outside_truncation_voxels = 0;
        double truncation_distance = 2 * tsdf_layer->voxel_size();
        std::vector<double> abserror;

        for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = gt_ptcloud_.begin(); it != gt_ptcloud_.end(); ++it) {
            voxblox::Point point(it->x, it->y, it->z);
            voxblox::FloatingPoint distance = 0.0;

            float weight = 0.0;
            const float min_weight = 0.01;
            const bool interpolate = true;
            // We will do multiple lookups -- the first is to determine whether the
            // voxel exists.
            if (!interpolator->getNearestDistanceAndWeight(point, &distance,
                                                            &weight)) {
                unknown_voxels++;
            } else if (weight <= min_weight) {
                unknown_voxels++;
            } else {
                if (distance >= truncation_distance) {
                    outside_truncation_voxels++;
                    distance = truncation_distance;
                } else {
                    // In case this fails, distance is still the nearest neighbor distance.
                    interpolator->getDistance(point, &distance, interpolate);
                }
                abserror.push_back(std::abs(distance));
                if (p_create_meshes_) {
                    voxblox::Layer<voxblox::TsdfVoxel>::BlockType::Ptr block_ptr =
                            tsdf_layer->getBlockPtrByCoordinates(point);
                    if (block_ptr != nullptr) {
                        voxblox::TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(point);
                        double frac = std::fabs(distance) / truncation_distance;
                        double r = std::min((frac - 0.5) * 2.0 + 1.0, 1.0) * 255;
                        double g = (1.0 - frac) * 2 * 255;
                        if (frac <= 0.5){
                            g = 190 + 130.0 * frac;
                        }
                        voxel.color = voxblox::Color(r, g, 0);
                    }
                }
            }
            total_evaluated_voxels++;
        }

        // create eror mesh
        if (p_create_meshes_){
            mesh_layer.reset(new voxblox::MeshLayer(tsdf_layer->block_size()));
            mesh_integrator.reset(new voxblox::MeshIntegrator<voxblox::TsdfVoxel>(
                    mesh_config, tsdf_layer.get(), mesh_layer.get()));
            mesh_integrator->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
            voxblox::outputMeshLayerAsPly(p_target_dir_ + "/meshes/" +map_name + "error.ply", *mesh_layer);
        }

        std::ostringstream result("");
        if (p_evaluate_){
            double mean = 0.0;
            for (int i = 0; i < abserror.size(); ++i){
                mean += abserror[i];
            }
            mean = mean / (total_evaluated_voxels - unknown_voxels);
            double stddev = 0.0;
            for (int i = 0; i < abserror.size(); ++i){
                stddev += std::pow(abserror[i]-mean, 2.0);
            }
            stddev = sqrt(stddev / (total_evaluated_voxels - unknown_voxels-1.0));
            // Build result
            result << mean << "," << stddev << "," << static_cast<double>(unknown_voxels) / total_evaluated_voxels << ","
                << outside_truncation_voxels / static_cast<double>(total_evaluated_voxels);
        }
        return result.str();
    }

    void EvaluationNode::writeLog(std::string text) {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        log_file_ << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S] ") << text << "\n";
    }

} // namespace mav_active_3d_planning

int main(int argc, char** argv) {
    ros::init(argc, argv, "evaluation_node");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::EvaluationNode eval(nh, nh_private);
    ros::spin();
    return 0;
}