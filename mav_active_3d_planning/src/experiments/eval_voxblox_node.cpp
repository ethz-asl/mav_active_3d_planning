#include <ros/ros.h>
#include <ros/console.h>
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

    // Ros-encapsulation for c++ voxblox code to evaluates voxblox maps against a gt pointcloud upon service-request.
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
        ROS_INFO("Evaluating target '%s'", p_target_dir_.c_str());
        log_file_path_ = p_target_dir_+ "/data_log.txt";
        std::ifstream read_log(log_file_path_.c_str());
        std::ifstream data_file((p_target_dir_+ "/voxblox_data.csv").c_str());
        if( !(read_log.is_open() && data_file.is_open())){
            ROS_ERROR("Unable to load the data and/or log files.");
            return false;
        }

        // Check not previously evaluated
        std::string line;
        while (std::getline(read_log, line)) {
            std::cout << "Read line '" + line + "'.\n";
            if (line == "[FLAG] Evaluated") {
                ROS_INFO("This file was already evaluated.");
                return true;
            }
        }

        // Setup log file
        read_log.close();
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
        std::stringstream buffer;
        std::string map_name;
        int n_maps = 0;
        while (std::getline(data_file, line)){
            line.pop_back();
            map_name = line.substr(0, line.find(","));
            if (map_name == "MapName") {
                map_name = "Header";
            } else if (map_name == "Unit") {
                map_name = "Unit";
            } else {
                n_maps++;
            }
            ROS_INFO("Processing: %s", map_name.c_str());
            buffer << line + "," + evaluateSingle(map_name) +"\n";
        }
        data_file.close();
        std::ofstream fout((p_target_dir_+ "/voxblox_data.csv").c_str(), std::ios::out);
        fout << buffer.rdbuf();
        fout.close();
        ROS_INFO("Evaluation finished.");

        // Finish
        writeLog("Evaluated "+std::to_string(n_maps)+" voxblox maps for fields 'RMSE, UnknownVoxels, OutsideTruncation'.");
        log_file_ << "[FLAG] Evaluated\n";
        log_file_.close();
        return true;
    }

    std::string EvaluationNode::evaluateSingle(std::string map_name) {
        if (map_name == "Header"){
            return "RMSE,UnknownVoxels,OutsideTruncation";
        } else if (map_name == "Unit"){
            return "meters,percent,percent";
        }

        // Setup the voxblox map and interpolater
        std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
        voxblox::Interpolator<voxblox::TsdfVoxel>::Ptr interpolator;
        voxblox::io::LoadLayer<voxblox::TsdfVoxel>(p_target_dir_ + "/voxblox_maps/" + map_name + ".vxblx", &tsdf_layer);
        interpolator.reset(new voxblox::Interpolator<voxblox::TsdfVoxel>(tsdf_layer.get()));

        // Go through each point, use trilateral interpolation to figure out the distance at that point.
        // This double-counts -- multiple queries to the same voxel will be counted separately.
        uint64_t total_evaluated_voxels = 0;
        uint64_t unknown_voxels = 0;
        uint64_t outside_truncation_voxels = 0;

        double truncation_distance = 2 * tsdf_layer->voxel_size();
        double mse = 0.0;

        for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = gt_ptcloud_.begin(); it != gt_ptcloud_.end(); ++it) {
            voxblox::Point point(it->x, it->y, it->z);
            voxblox::FloatingPoint distance = 0.0;

            float weight = 0.0;
            bool valid = false;

            const float min_weight = 0.01;
            const bool interpolate = true;
            // We will do multiple lookups -- the first is to determine whether the
            // voxel exists.
            if (!interpolator->getNearestDistanceAndWeight(point, &distance,
                                                            &weight)) {
                unknown_voxels++;
            } else if (weight <= min_weight) {
                unknown_voxels++;
            } else if (distance >= truncation_distance) {
                outside_truncation_voxels++;
                mse += truncation_distance * truncation_distance;
                valid = true;
            } else {
                // In case this fails, distance is still the nearest neighbor distance.
                interpolator->getDistance(point, &distance, interpolate);
                mse += distance * distance;
                valid = true;
            }
            total_evaluated_voxels++;
        }

        double rms = sqrt(mse / (total_evaluated_voxels - unknown_voxels));

        // Build result
        std::ostringstream result("");
        result << rms << "," << static_cast<double>(unknown_voxels) / total_evaluated_voxels << ","
            << outside_truncation_voxels / static_cast<double>(total_evaluated_voxels);
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
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    mav_active_3d_planning::EvaluationNode eval(nh, nh_private);
    ros::spin();
    return 0;
}