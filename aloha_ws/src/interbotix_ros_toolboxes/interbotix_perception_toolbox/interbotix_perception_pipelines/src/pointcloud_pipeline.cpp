// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/crop_box.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "interbotix_perception_msgs/msg/cluster_info.hpp"
#include "interbotix_perception_msgs/srv/filter_params.hpp"
#include "interbotix_perception_msgs/srv/cluster_info_array.hpp"

typedef pcl::PointXYZRGB PointT;

using SetBool = std_srvs::srv::SetBool;
using Marker = visualization_msgs::msg::Marker;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using ClusterInfo = interbotix_perception_msgs::msg::ClusterInfo;
using FilterParams = interbotix_perception_msgs::srv::FilterParams;
using ClusterInfoArray = interbotix_perception_msgs::srv::ClusterInfoArray;

// ROS2 node
rclcpp::Node::SharedPtr node_;

rclcpp::Logger LOGGER = rclcpp::get_logger("pointcloud_pipeline");

// // Object PointCloud Publisher
rclcpp::Publisher<PointCloud2>::SharedPtr pub_pc_obj;

// Filtered PointCloud2 Publisher
rclcpp::Publisher<PointCloud2>::SharedPtr pub_pc_filter;

// Object Marker Publisher
rclcpp::Publisher<Marker>::SharedPtr pub_marker_obj;

// Cropbox Marker Publisher
rclcpp::Publisher<Marker>::SharedPtr pub_marker_crop;

// FilterParams Service Server
rclcpp::Service<FilterParams>::SharedPtr service_set_filter_params;

// ClusterInfoArray Service Server
rclcpp::Service<ClusterInfoArray>::SharedPtr service_get_clusters;

// Enable Pipeline Service Server
rclcpp::Service<SetBool>::SharedPtr service_enable_pipeline;

// Raw PointCloud2 Subscriber
rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud;

// Object Marker Message
Marker marker_obj;

// Cropbox Marker Message
Marker marker_crop;

// Raw PointCloud2 input message
PointCloud2::SharedPtr input;

// Vector of ClusterInfo messages
std::vector<ClusterInfo> cluster_info_vector;

// Whether or not the pipeline is enabled
bool _enable_pipeline = true;

// Minimum position of the CropBox filter
float x_filter_min, y_filter_min, z_filter_min;

// Maximum position of the CropBox filter
float x_filter_max, y_filter_max, z_filter_max;

// Voxel Leaf Size parameter in meters
float voxel_leaf_size;

// Distance Threshold for the plane segmentation in meters
float plane_dist_thresh;

// Radius Outlier removal distance in meters
float ror_radius_search;

// Distance from cluster in which to add points
float cluster_tol;

// Maximum number of iterations for the plane segmentation
int plane_max_iter;

// Minimum number of neighbors within ror radius to be considered
int ror_min_neighbors;

// Minimum number of points a cluster can contain
int cluster_min_size;

// Maximum number of points a cluster can contain
int cluster_max_size;

void pointcloud_pipeline()
{
  // raw cloud fron input
  pcl::PointCloud<PointT>::Ptr cloud_raw(new pcl::PointCloud<PointT>);

  // downsampled cloud via voxelgrid filter
  pcl::PointCloud<PointT>::Ptr cloud_voxel_downsampled(new pcl::PointCloud<PointT>);

  // cropped cloud via cropbox
  pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>);

  // cloud without place via segmentation filter
  pcl::PointCloud<PointT>::Ptr cloud_no_plane(new pcl::PointCloud<PointT>);

  // de-noised cloud via radius outlier removal filter
  pcl::PointCloud<PointT>::Ptr cloud_no_noise(new pcl::PointCloud<PointT>);

  // Convert to pcl point cloud
  pcl::fromROSMsg(*input, *cloud_raw);
  if (cloud_raw->empty()) {
    return;
  }

  // Cropbox Filter
  pcl::CropBox<PointT> crop;
  crop.setInputCloud(cloud_raw);
  Eigen::Vector4f min_point = Eigen::Vector4f(x_filter_min, y_filter_min, z_filter_min, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(x_filter_max, y_filter_max, z_filter_max, 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  crop.filter(*cloud_cropped);
  if (cloud_cropped->empty()) {
    return;
  }

  // Downsample via the VoxelFilter
  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud_cropped);
  vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  vg.filter(*cloud_voxel_downsampled);
  if (cloud_voxel_downsampled->size() < 4) {
    return;
  }

  // Convert to ROS PointCloud2 msg and publish quasi-filtered pointcloud (for debugging)
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_voxel_downsampled, output);
  pub_pc_filter->publish(output);

  // Publish Semi-transparent cube outlining the crop box filter's borders
  marker_crop.header.frame_id = input->header.frame_id;
  marker_crop.pose.position.x = (x_filter_min + x_filter_max) / 2.0;
  marker_crop.pose.position.y = (y_filter_min + y_filter_max) / 2.0;
  marker_crop.pose.position.z = (z_filter_min + z_filter_max) / 2.0;
  marker_crop.scale.x = x_filter_max - x_filter_min;
  marker_crop.scale.y = y_filter_max - y_filter_min;
  marker_crop.scale.z = z_filter_max - z_filter_min;
  pub_marker_crop->publish(marker_crop);

  // Do segmentation of the table
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<PointT> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(plane_max_iter);
  seg.setDistanceThreshold(plane_dist_thresh);
  seg.setInputCloud(cloud_voxel_downsampled);
  seg.segment(*inliers, *coefficients);

  // Extract indices of the segmented plane and remove it from the cloud
  pcl::ExtractIndices<PointT> ei;
  ei.setInputCloud(cloud_voxel_downsampled);
  ei.setIndices(inliers);
  ei.setNegative(true);
  ei.filter(*cloud_no_plane);
  if (cloud_no_plane->empty()) {
    return;
  }

  // Radius Outlier Removal (removes random noise in the cloud)
  pcl::RadiusOutlierRemoval<PointT> ror;
  ror.setInputCloud(cloud_no_plane);
  ror.setRadiusSearch(ror_radius_search);
  ror.setMinNeighborsInRadius(ror_min_neighbors);
  ror.setKeepOrganized(false);
  ror.filter(*cloud_no_noise);
  if (cloud_no_noise->empty()) {
    return;
  }

  // Publish the final processed pointcloud
  sensor_msgs::msg::PointCloud2 output_2;
  pcl::toROSMsg(*cloud_no_noise, output_2);
  pub_pc_obj->publish(output_2);

  // Extract Clusters
  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud_no_noise);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(cluster_tol);  // 2cm
  ec.setMinClusterSize(cluster_min_size);
  ec.setMaxClusterSize(cluster_max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_no_noise);
  ec.extract(cluster_indices);

  // RCLCPP_INFO(LOGGER, "Found %d clusters.", cluster_indices.size());

  int j = 0;
  cluster_info_vector.clear();
  for (
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
    it != cluster_indices.end();
    ++it)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (
      std::vector<int>::const_iterator pit = it->indices.begin();
      pit != it->indices.end();
      ++pit)
    {
      cloud_cluster->push_back((*cloud_no_noise)[*pit]);
    }
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    // Compute the centroid of each cluster. Then find the point with the min 'z' value. This point
    // is assumed to be at the top of the pointcloud cluster, and can be used to make sure the arm
    // doesn't go too far down when grasping
    PointT cntrd, min_point;
    pcl::computeCentroid(*cloud_cluster, cntrd);
    min_point = cloud_cluster->points[0];
    for (size_t indx {1}; indx < cloud_cluster->size(); indx++) {
      if (cloud_cluster->points[indx].z < min_point.z) {
        min_point = cloud_cluster->points[indx];
      }
    }

    // Publish a spherical marker showing the centroid of the cluster
    marker_obj.id = j;
    marker_obj.header.frame_id = input->header.frame_id;
    marker_obj.pose.position.x = cntrd.x;
    marker_obj.pose.position.y = cntrd.y;
    marker_obj.pose.position.z = cntrd.z;
    marker_obj.color.r = cntrd.r / 255.0;
    marker_obj.color.g = cntrd.g / 255.5;
    marker_obj.color.b = cntrd.b / 255.0;
    marker_obj.header.stamp = node_->get_clock()->now();
    pub_marker_obj->publish(marker_obj);

    // Save a ClusterInfo message with info on the cluster
    ClusterInfo ci_msg;
    ci_msg.frame_id = input->header.frame_id;
    ci_msg.position.x = cntrd.x;
    ci_msg.position.y = cntrd.y;
    ci_msg.position.z = cntrd.z;
    ci_msg.color.r = cntrd.r;
    ci_msg.color.g = cntrd.g;
    ci_msg.color.b = cntrd.b;
    ci_msg.min_z_point.x = min_point.x;
    ci_msg.min_z_point.y = min_point.y;
    ci_msg.min_z_point.z = min_point.z;
    ci_msg.num_points = cloud_cluster->size();
    cluster_info_vector.push_back(ci_msg);
    j++;
  }
  RCLCPP_DEBUG(LOGGER, "Found %ld clusters.", cluster_info_vector.size());
}

void enable_pipeline(bool enable)
{
  _enable_pipeline = enable;
}

void cloud_cb(const PointCloud2::SharedPtr pointcloud)
{
  input = pointcloud;

  if (_enable_pipeline) {
    pointcloud_pipeline();
  }
}

bool srv_set_filter_params(
  const std::shared_ptr<rmw_request_id_t> request_header,
  FilterParams::Request::SharedPtr req,
  FilterParams::Response::SharedPtr res)
{
  // avoid unused parameter warnings
  (void)res;
  (void)request_header;
  voxel_leaf_size = req->voxel_leaf_size;
  x_filter_min = req->x_filter_min;
  x_filter_max = req->x_filter_max;
  y_filter_min = req->y_filter_min;
  y_filter_max = req->y_filter_max;
  z_filter_min = req->z_filter_min;
  z_filter_max = req->z_filter_max;
  plane_max_iter = req->plane_max_iter;
  plane_dist_thresh = req->plane_dist_thresh;
  ror_radius_search = req->ror_radius_search;
  ror_min_neighbors = req->ror_min_neighbors;
  cluster_tol = req->cluster_tol;
  cluster_min_size = req->cluster_min_size;
  cluster_max_size = req->cluster_max_size;
  return true;
}

bool srv_get_cluster_positions(
  std::shared_ptr<rmw_request_id_t> request_header,
  ClusterInfoArray::Request::SharedPtr req,
  ClusterInfoArray::Response::SharedPtr res)
{
  // avoid unused parameter warnings
  (void)request_header;
  (void)req;
  if (!_enable_pipeline) {
    pointcloud_pipeline();
  }

  res->clusters = cluster_info_vector;
  return true;
}

bool srv_enable_pipeline(
  std::shared_ptr<rmw_request_id_t> request_header,
  SetBool::Request::SharedPtr req,
  SetBool::Response::SharedPtr res)
{
  // avoid unused parameter warnings
  (void)request_header;
  (void)res;
  enable_pipeline(req->data);
  return true;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node_ = std::make_shared<rclcpp::Node>("pc_filter");

  std::string cloud_topic;
  node_->declare_parameter<bool>("enable_pipeline", true);
  node_->declare_parameter<std::string>("cloud_topic", "/camera/depth/color/points");
  node_->declare_parameter<float>("voxel_leaf_size", 0.004);
  node_->declare_parameter<float>("x_filter_min", -0.25);
  node_->declare_parameter<float>("y_filter_min", -0.25);
  node_->declare_parameter<float>("z_filter_min", 0.25);
  node_->declare_parameter<float>("x_filter_max", 0.25);
  node_->declare_parameter<float>("y_filter_max", 0.25);
  node_->declare_parameter<float>("z_filter_max", 0.75);
  node_->declare_parameter<int>("plane_max_iter", 50);
  node_->declare_parameter<float>("plane_dist_thresh", 0.005);
  node_->declare_parameter<float>("ror_radius_search", 0.01);
  node_->declare_parameter<int>("ror_min_neighbors", 5);
  node_->declare_parameter<float>("cluster_tol", 0.02);
  node_->declare_parameter<int>("cluster_min_size", 50);
  node_->declare_parameter<int>("cluster_max_size", 1000);

  node_->get_parameter("enable_pipeline", _enable_pipeline);
  node_->get_parameter("cloud_topic", cloud_topic);
  node_->get_parameter("voxel_leaf_size", voxel_leaf_size);
  node_->get_parameter("x_filter_min", x_filter_min);
  node_->get_parameter("y_filter_min", y_filter_min);
  node_->get_parameter("z_filter_min", z_filter_min);
  node_->get_parameter("x_filter_max", x_filter_max);
  node_->get_parameter("y_filter_max", y_filter_max);
  node_->get_parameter("z_filter_max", z_filter_max);
  node_->get_parameter("plane_max_iter", plane_max_iter);
  node_->get_parameter("plane_dist_thresh", plane_dist_thresh);
  node_->get_parameter("ror_radius_search", ror_radius_search);
  node_->get_parameter("ror_min_neighbors", ror_min_neighbors);
  node_->get_parameter("cluster_tol", cluster_tol);
  node_->get_parameter("cluster_min_size", cluster_min_size);
  node_->get_parameter("cluster_max_size", cluster_max_size);

  // Initialize ROS Services, Subscribers, and Publishers
  service_set_filter_params = node_->create_service<FilterParams>(
    "set_filter_params",
    srv_set_filter_params);
  service_get_clusters = node_->create_service<ClusterInfoArray>(
    "get_cluster_positions",
    srv_get_cluster_positions);
  service_enable_pipeline = node_->create_service<SetBool>(
    "enable_pipeline",
    srv_enable_pipeline);

  sub_pointcloud = node_->create_subscription<PointCloud2>(
    cloud_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)),
    cloud_cb);

  pub_pc_obj = node_->create_publisher<PointCloud2>("pointcloud/objects", 1);
  pub_pc_filter = node_->create_publisher<PointCloud2>("pointcloud/filtered", 1);
  pub_marker_obj = node_->create_publisher<Marker>("markers/objects", 50);
  pub_marker_crop = node_->create_publisher<Marker>("markers/crop_box", 1);

  // Populate the marker object used to show the centroid of each cluster with values that will
  // never be changed
  marker_obj.type = Marker::SPHERE;
  marker_obj.action = Marker::ADD;
  using namespace std::chrono_literals;
  marker_obj.lifetime = rclcpp::Duration(0.1s);
  marker_obj.pose.orientation.x = 0.0;
  marker_obj.pose.orientation.y = 0.0;
  marker_obj.pose.orientation.z = 0.0;
  marker_obj.pose.orientation.w = 1.0;
  marker_obj.color.a = 1.0;
  marker_obj.scale.x = 0.01;
  marker_obj.scale.y = 0.01;
  marker_obj.scale.z = 0.01;

  // Populate Crop Box limits with values that will never be changed
  marker_crop.type = Marker::CUBE;
  marker_crop.action = Marker::ADD;
  marker_crop.pose.orientation.x = 0.0;
  marker_crop.pose.orientation.y = 0.0;
  marker_crop.pose.orientation.z = 0.0;
  marker_crop.pose.orientation.w = 1.0;
  marker_crop.color.g = 1.0;
  marker_crop.color.a = 0.3;

  RCLCPP_INFO(LOGGER, "InterbotixPointCloudPipeline is up!");

  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}
