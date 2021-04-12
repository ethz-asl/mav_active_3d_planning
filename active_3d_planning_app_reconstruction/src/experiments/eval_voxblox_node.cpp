#include <stdio.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
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

#include "active_3d_planning_core/libs/nanoflann.hpp"

namespace active_3d_planning {

// Ros-wrapper for c++ voxblox code to evaluates voxblox maps upon request from
// the eval_plotting_node. Largely based on the voxblox_ros/voxblox_eval.cc
// code. Pretty ugly and non-general code but just needs to work in this
// specific case atm...
class EvaluationNode {
 public:
  EvaluationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool evaluate(std_srvs::Empty::Request& req,    // NOLINT
                std_srvs::Empty::Response& res);  // NOLINT

  std::string evaluateSingle(std::string map_name, bool create_mesh);

  void writeLog(std::string text);

  // nanoflann pcl adapter
  struct TreeData {
    // data
    std::vector<Eigen::Vector3f> points;

    // nanoflann functionality
    inline std::size_t kdtree_get_point_count() const { return points.size(); }

    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
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

  // kdtree
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<float, TreeData>, TreeData, 3>
      KDTree;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::ServiceServer eval_srv_;

  // params
  std::string p_target_dir_;
  bool p_create_meshes_;
  bool p_evaluate_;
  bool p_error_histogram_;  // create a histo with 20 bins
  bool p_evaluate_volume_;  // true evaluate explrored volume in target bounding
  // volume
  double p_truncation_distance_;  // leave 0.0 for 3x voxelsize
  int p_mesh_steps_;
  double p_target_bounding_volume_[6];  // for volume evaluation, x_min, x_max,
                                        // ... y, z

  // variables
  int mesh_counter_;
  int hist_bins_;

  // log file
  std::string log_file_path_;
  std::ofstream log_file_;
  std::ofstream hist_file_;

  // Ground truth pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> gt_ptcloud_;
  KDTree* kdtree_;
  TreeData kdtree_data_;
};

EvaluationNode::EvaluationNode(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), mesh_counter_(0), hist_bins_(30) {
  eval_srv_ =
      nh_private_.advertiseService("evaluate", &EvaluationNode::evaluate, this);
}

bool EvaluationNode::evaluate(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& res) {
  // Get target directory
  nh_private_.getParam("target_directory", p_target_dir_);
  ROS_INFO("Starting voxblox evaluation.");
  log_file_path_ = p_target_dir_ + "/data_log.txt";
  std::ifstream read_log(log_file_path_.c_str());
  std::ifstream data_file((p_target_dir_ + "/voxblox_data.csv").c_str());
  if (!(read_log.is_open() && data_file.is_open())) {
    ROS_ERROR("Unable to load the data and/or log files.");
    read_log.close();
    data_file.close();
    return false;
  }

  // Get actions to perform
  nh_private_.param("evaluate", p_evaluate_, true);
  nh_private_.param("create_meshes", p_create_meshes_, true);
  nh_private_.param("mesh_every_n_steps", p_mesh_steps_, 20);
  nh_private_.param("truncation_distance", p_truncation_distance_, 0.0);
  nh_private_.param("error_histogram", p_error_histogram_, true);
  nh_private_.param("evaluate_volume", p_evaluate_volume_, true);
  nh_private_.param("evaluation_bounding_volum/x_min",
                    p_target_bounding_volume_[0], 0.0);
  nh_private_.param("evaluation_bounding_volum/x_max",
                    p_target_bounding_volume_[1], 0.0);
  nh_private_.param("evaluation_bounding_volum/y_min",
                    p_target_bounding_volume_[2], 0.0);
  nh_private_.param("evaluation_bounding_volum/y_max",
                    p_target_bounding_volume_[3], 0.0);
  nh_private_.param("evaluation_bounding_volum/z_min",
                    p_target_bounding_volume_[4], 0.0);
  nh_private_.param("evaluation_bounding_volum/z_max",
                    p_target_bounding_volume_[5], 0.0);

  // Check not previously evaluated
  std::string line;
  if (p_evaluate_ || p_evaluate_volume_) {
    while (std::getline(read_log, line)) {
      if (line == "[FLAG] Evaluated") {
        ROS_INFO("This file was already evaluated.");
        p_evaluate_ = false;
        p_evaluate_volume_ = false;
      }
    }
  }
  read_log.close();
  if (!p_evaluate_ && !p_create_meshes_ && !p_evaluate_volume_) {
    return true;
  }

  // Setup log file
  log_file_.open(log_file_path_.c_str(), std::ios::app);

  // Load the groundtruth pointcloud.
  std::string gt_path;
  nh_private_.getParam("ground_truth_path", gt_path);
  pcl::PLYReader ply_reader;
  kdtree_ = new KDTree(3, kdtree_data_,
                       nanoflann::KDTreeSingleIndexAdaptorParams(10));
  if (ply_reader.read(gt_path, gt_ptcloud_) != 0) {
    ROS_ERROR("Unable to load ground truth pointcloudfrom '%s'.",
              gt_path.c_str());
    p_evaluate_ = false;
    p_create_meshes_ = false;
    p_error_histogram_ = false;
    if (!p_evaluate_volume_) {
      return false;
    }
  } else {
    ROS_INFO("Succesfully loaded ground truth pointcloud from '%s'.",
             gt_path.c_str());
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it =
             gt_ptcloud_.begin();
         it != gt_ptcloud_.end(); ++it) {
      kdtree_data_.points.push_back(Eigen::Vector3f(it->x, it->y, it->z));
    }
    kdtree_->buildIndex();
  }

  // Setup hist file
  if (p_error_histogram_ && p_evaluate_) {
    hist_file_.open((p_target_dir_ + "/error_hist.csv").c_str(), std::ios::out);
    hist_file_ << "MapName";
    for (int i = 0; i < hist_bins_; ++i) {
      hist_file_ << ",Bin" << i;
    }
    hist_file_ << "\n";
  }

  // Parse and evaluate all lines in the data file
  std::string map_name;
  int n_maps = 0;
  std::ofstream fout;
  if (p_evaluate_ || p_evaluate_volume_) {
    fout.open((p_target_dir_ + "/voxblox_data_temp.csv").c_str(),
              std::ios::out);
  }
  ROS_INFO("Start processing maps.");
  while (std::getline(data_file, line)) {
    map_name = line.substr(0, line.find(","));
    if (map_name == "MapName") {
      map_name = "Header";
    } else if (map_name == "Unit") {
      map_name = "Unit";
    } else {
      // Check map exists
      if (!std::ifstream(
              (p_target_dir_ + "/voxblox_maps/" + map_name + ".vxblx")
                  .c_str())) {
        ROS_INFO("Skipping map '%s' (non-existant)", map_name.c_str());
        if (p_evaluate_ || p_evaluate_volume_) {
          fout << line + ",0,0,0,0,0,0,0\n";
        }
        continue;
      }
      n_maps++;
      mesh_counter_++;
    }
    ROS_INFO("Processing: %s", map_name.c_str());
    bool create_mesh = false;
    if (p_create_meshes_ && mesh_counter_ >= p_mesh_steps_) {
      create_mesh = true;
      mesh_counter_ = 0;
    }
    std::string result = evaluateSingle(map_name, create_mesh);
    if (p_evaluate_ || p_evaluate_volume_) {
      fout << line + "," + result + "\n";
    }
  }
  data_file.close();
  if (p_evaluate_ || p_evaluate_volume_) {
    fout.close();
  }
  if (p_error_histogram_) {
    hist_file_.close();
  }
  ROS_INFO("Finished processing maps.");

  // Finish
  if (p_evaluate_ || p_evaluate_volume_) {
    if (std::rename((p_target_dir_ + "/voxblox_data_temp.csv").c_str(),
                    (p_target_dir_ + "/voxblox_data.csv").c_str()) != 0) {
      ROS_ERROR("Could not rename temp csv file!");
    }
    writeLog("Evaluated " + std::to_string(n_maps) +
             " voxblox maps. Fields: 'MeanError', 'StdDevError', 'RMSE', "
             "'UnknownVoxels', 'OutsideTruncation', 'Volume'.");
    log_file_ << "[FLAG] Evaluated\n";
  }
  if (p_create_meshes_) {
    writeLog("Created " + std::to_string(n_maps) + " color and error meshes.");
  }
  log_file_.close();
  ROS_INFO("Voxblox evaluation finished succesfully.");
  return true;
}

std::string EvaluationNode::evaluateSingle(std::string map_name,
                                           bool create_mesh) {
  if (map_name == "Header") {
    return "MeanError,StdDevError,RMSE,UnknownVoxels,OutsideTruncation,Volume";
  } else if (map_name == "Unit") {
    return "meters,meters,meters,percent,percent,m3";
  }

  // Setup the voxblox map and interpolater
  std::shared_ptr<voxblox::Layer<voxblox::TsdfVoxel>> tsdf_layer;
  voxblox::Interpolator<voxblox::TsdfVoxel>::Ptr interpolator;
  voxblox::io::LoadLayer<voxblox::TsdfVoxel>(
      p_target_dir_ + "/voxblox_maps/" + map_name + ".vxblx", &tsdf_layer);
  interpolator.reset(
      new voxblox::Interpolator<voxblox::TsdfVoxel>(tsdf_layer.get()));

  // create color mesh
  voxblox::MeshIntegratorConfig mesh_config;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer;
  std::shared_ptr<voxblox::MeshIntegrator<voxblox::TsdfVoxel>> mesh_integrator;
  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  if (create_mesh) {
    mesh_layer.reset(new voxblox::MeshLayer(tsdf_layer->block_size()));
    mesh_integrator.reset(new voxblox::MeshIntegrator<voxblox::TsdfVoxel>(
        mesh_config, tsdf_layer.get(), mesh_layer.get()));
    mesh_integrator->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    voxblox::outputMeshLayerAsPly(
        p_target_dir_ + "/meshes/" + map_name + "color.ply", *mesh_layer);
  }

  // Go through each point, use trilateral interpolation to figure out the
  // distance at that point. This double-counts -- multiple queries to the same
  // voxel will be counted separately.
  uint64_t total_evaluated_voxels = 0;
  uint64_t unknown_voxels = 0;
  uint64_t outside_truncation_voxels = 0;
  const float min_weight = 0.01;
  const bool interpolate = true;
  double truncation_distance = 3 * tsdf_layer->voxel_size();
  if (p_truncation_distance_ != 0.0) {
    truncation_distance = p_truncation_distance_;
  }
  std::vector<double> abserror;
  if (p_evaluate_) {
    // Evaluate gt pcl based(# gt points within < trunc_dist)
    for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it =
             gt_ptcloud_.begin();
         it != gt_ptcloud_.end(); ++it) {
      voxblox::Point point(it->x, it->y, it->z);
      voxblox::FloatingPoint distance = 0.0;

      float weight = 0.0;
      // We will do multiple lookups -- the first is to determine whether the
      // voxel exists.
      if (!interpolator->getNearestDistanceAndWeight(point, &distance,
                                                     &weight)) {
        unknown_voxels++;
      } else if (weight <= min_weight) {
        unknown_voxels++;
      } else {
        interpolator->getDistance(point, &distance, interpolate);
        if (std::abs(distance) > truncation_distance) {
          distance = truncation_distance;
        }
        abserror.push_back(std::abs(distance));
      }
      total_evaluated_voxels++;
    }
  }

  // Evaluate volume
  double volume = 0.0;
  if (p_evaluate_volume_) {
    voxblox::BlockIndexList blocks;
    tsdf_layer->getAllAllocatedBlocks(&blocks);
    const size_t vps = tsdf_layer->voxels_per_side();
    const size_t num_voxels_per_block = vps * vps * vps;
    const double dv = std::pow(tsdf_layer->voxel_size(), 3);
    for (voxblox::BlockIndex& index : blocks) {
      // Iterate over all voxels in said blocks.
      voxblox::Block<voxblox::TsdfVoxel>& block =
          tsdf_layer->getBlockByIndex(index);
      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
           ++linear_index) {
        voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        // Voxel parsing
        voxblox::Point voxel_center =
            block.computeCoordinatesFromLinearIndex(linear_index);
        if (voxel.weight < min_weight) {
          continue;
        } else if (voxel_center.x() < p_target_bounding_volume_[0]) {
          continue;
        } else if (voxel_center.x() > p_target_bounding_volume_[1]) {
          continue;
        } else if (voxel_center.y() < p_target_bounding_volume_[2]) {
          continue;
        } else if (voxel_center.y() > p_target_bounding_volume_[3]) {
          continue;
        } else if (voxel_center.z() < p_target_bounding_volume_[4]) {
          continue;
        } else if (voxel_center.z() > p_target_bounding_volume_[5]) {
          continue;
        }
        volume += dv;
      }
    }
  }

  // Coloring: grey -> unknown, green -> 0 error, red -> maximum error
  // (truncation dist), purple: >truncation
  if (create_mesh) {
    voxblox::BlockIndexList blocks;
    tsdf_layer->getAllAllocatedBlocks(&blocks);
    const size_t vps = tsdf_layer->voxels_per_side();
    const size_t num_voxels_per_block = vps * vps * vps;
    for (voxblox::BlockIndex& index : blocks) {
      // Iterate over all voxels in said blocks.
      voxblox::Block<voxblox::TsdfVoxel>& block =
          tsdf_layer->getBlockByIndex(index);
      for (size_t linear_index = 0; linear_index < num_voxels_per_block;
           ++linear_index) {
        voxblox::TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
        // Voxel parsing
        voxblox::Point voxel_center =
            block.computeCoordinatesFromLinearIndex(linear_index);
        float query_pt[3] = {voxel_center.x(), voxel_center.y(),
                             voxel_center.z()};
        std::size_t ret_index;
        float out_dist_sqr;
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        kdtree_->findNeighbors(resultSet, &query_pt[0],
                               nanoflann::SearchParams(10));

        if (out_dist_sqr > truncation_distance * truncation_distance) {
          // voxel is not near the ground truth
          voxel.color = voxblox::Color(128, 128, 128);
          continue;
        }

        // get error of closest gt point
        voxblox::Point point(kdtree_data_.points[ret_index].x(),
                             kdtree_data_.points[ret_index].y(),
                             kdtree_data_.points[ret_index].z());
        voxblox::FloatingPoint distance = 0.0;
        float weight = 0.0;
        if (!interpolator->getNearestDistanceAndWeight(point, &distance,
                                                       &weight)) {
          voxel.color = voxblox::Color(128, 128, 128);
        } else if (weight <= min_weight) {
          voxel.color = voxblox::Color(128, 128, 128);
        } else {
          interpolator->getDistance(point, &distance, interpolate);
          if (distance >= truncation_distance) {
            voxel.color = voxblox::Color(255, 0, 0);
          } else {
            // In case this fails, distance is still the nearest neighbor
            // distance.
            double frac = std::abs(distance) / truncation_distance;
            double r = std::min((frac - 0.5) * 2.0 + 1.0, 1.0) * 255;
            double g = (1.0 - frac) * 2 * 255;
            if (frac <= 0.5) {
              g = 190 + 130.0 * frac;
            }
            voxel.color = voxblox::Color(r, g, 0);
          }
        }
      }
    }

    // produce the now colored mesh
    mesh_layer.reset(new voxblox::MeshLayer(tsdf_layer->block_size()));
    mesh_integrator.reset(new voxblox::MeshIntegrator<voxblox::TsdfVoxel>(
        mesh_config, tsdf_layer.get(), mesh_layer.get()));
    mesh_integrator->generateMesh(only_mesh_updated_blocks, clear_updated_flag);
    voxblox::outputMeshLayerAsPly(
        p_target_dir_ + "/meshes/" + map_name + "error.ply", *mesh_layer);
  }

  std::ostringstream result("");
  // Evaluate result
  double mean = 0.0;
  double rmse = 0.0;
  for (int i = 0; i < abserror.size(); ++i) {
    mean += abserror[i];
    rmse += std::pow(abserror[i], 2);
  }
  if (total_evaluated_voxels - unknown_voxels > 0) {
    mean = mean / (total_evaluated_voxels - unknown_voxels);
    rmse = rmse / (total_evaluated_voxels - unknown_voxels);
    rmse = std::sqrt(rmse);
  }
  double stddev = 0.0;
  for (int i = 0; i < abserror.size(); ++i) {
    stddev += std::pow(abserror[i] - mean, 2.0);
  }
  if (total_evaluated_voxels - unknown_voxels > 1) {
    stddev = sqrt(stddev / (total_evaluated_voxels - unknown_voxels - 1.0));
  }
  double outside = 0.0;
  double unknown = 0.0;
  if (total_evaluated_voxels > 0) {
    outside =
        static_cast<double>(outside_truncation_voxels) / total_evaluated_voxels;
    unknown = static_cast<double>(unknown_voxels) / total_evaluated_voxels;
  }

  // Build result
  result << mean << "," << stddev << "," << rmse << "," << unknown << ","
         << outside << "," << volume;

  if (p_error_histogram_) {
    // create histogram of error distribution
    std::vector<int> histogram(hist_bins_);
    const double bin_size =
        truncation_distance / (static_cast<double>(hist_bins_) - 1.0);
    for (int i = 0; i < abserror.size(); ++i) {
      int bin = static_cast<int>(floor(abserror[i] / bin_size));
      if (bin < 0 || bin >= hist_bins_) {
        std::cout << "Bin Error at bin " << bin << ", value " << abserror[i]
                  << std::endl;
        continue;
      }
      histogram[bin] += 1;
    }
    hist_file_ << map_name;
    for (int i = 0; i < histogram.size(); ++i) {
      hist_file_ << "," << histogram[i];
    }
    hist_file_ << "\n";
  }
  return result.str();
}

void EvaluationNode::writeLog(std::string text) {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  log_file_ << std::put_time(&tm, "[%Y-%m-%d %H:%M:%S] ") << text << "\n";
}

}  // namespace active_3d_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "evaluation_node");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  active_3d_planning::EvaluationNode eval(nh, nh_private);
  ros::spin();
  return 0;
}
