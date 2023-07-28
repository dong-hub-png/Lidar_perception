#ifndef GROUND_SEGMENTATION__RANSAC_GROUND_FILTER_NODELET_HPP_
#define GROUND_SEGMENTATION__RANSAC_GROUND_FILTER_NODELET_HPP_

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/transform_datatypes.h>
// PCL includes
#include <boost/thread/mutex.hpp>

#include <pcl/filters/filter.h>
// PCL includes
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_msgs/msg/model_coefficients.h>
#include <pcl_msgs/msg/point_indices.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <string>
#include <vector>

namespace ground_segmentation
{
struct PlaneBasis
{
  Eigen::Vector3d e_x;
  Eigen::Vector3d e_y;
  Eigen::Vector3d e_z;
};


class RANSACGroundFilterComponent : public rclcpp::Node
{
public:

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;
using ModelCoefficients = pcl_msgs::msg::ModelCoefficients;
using ModelCoefficientsPtr = ModelCoefficients::SharedPtr;
using ModelCoefficientsConstPtr = ModelCoefficients::ConstSharedPtr;

using IndicesPtr = pcl::IndicesPtr;
using IndicesConstPtr = pcl::IndicesConstPtr;
using PointType = pcl::PointXYZ;


explicit  RANSACGroundFilterComponent(const rclcpp::NodeOptions & node_options);

/** \brief The input PointCloud2 subscriber. */
rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

/** \brief The output PointCloud2 publisher. */
rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

void RANSAC_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input);

bool transformPointCloud(
  const std::string & in_target_frame, const PointCloud2::SharedPtr & in_cloud_ptr,
  const PointCloud2::SharedPtr & out_cloud_ptr);

void applyRANSAC(
  const pcl::PointCloud<PointType>::Ptr & input, pcl::PointIndices::Ptr & output_inliers,
  pcl::ModelCoefficients::Ptr & output_coefficients);

void extractPointsIndices(
  const pcl::PointCloud<PointType>::Ptr in_cloud_ptr, const pcl::PointIndices & in_indices,
  pcl::PointCloud<PointType>::Ptr out_only_indices_cloud_ptr,
  pcl::PointCloud<PointType>::Ptr out_removed_indices_cloud_ptr);

Eigen::Affine3d getPlaneAffine(
  const pcl::PointCloud<PointType> segment_ground_cloud, const Eigen::Vector3d & plane_normal);

PlaneBasis getPlaneBasis(const Eigen::Vector3d & plane_normal);

Eigen::Vector3d getArbitraryOrthogonalVector(const Eigen::Vector3d & input);

std::string base_frame_ = "base_link";
std::string unit_axis_ = "z";
std::string input_topic;
std::string output_topic;
int max_iterations_ = 0;
int min_inliers_ = 0;
int min_points_ = 0;
double outlier_threshold_ = 0.1;
double plane_slope_threshold_ = 10.0;
double height_threshold_ = 0.1;
double voxel_size_x_ = 0.1;
double voxel_size_y_ = 0.1;
double voxel_size_z_ = 0.1;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
Eigen::Vector3d unit_vec_ = Eigen::Vector3d::UnitZ();
};
}
#endif  // GROUND_SEGMENTATION__RANSAC_GROUND_FILTER_NODELET_HPP_