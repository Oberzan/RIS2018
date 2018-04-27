#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/PointStamped.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher pubm;

tf2_ros::Buffer tf2_buffer;

typedef pcl::PointXYZRGB PointT;

void PassThroughFilter(pcl::PointCloud<PointT>::Ptr cloud)
{
  pcl::PassThrough<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("z");
  filter.setFilterLimits(0, 1.5);
  filter.filter(*cloud);
  filter.setInputCloud(cloud);
  filter.setFilterFieldName("y");
  filter.setFilterLimits(-0.15, 0.15);
  filter.filter(*cloud);
}

void StatisticalOutlierRemovalFilter(pcl::PointCloud<PointT>::Ptr cloud)
{
  pcl::StatisticalOutlierRemoval<PointT> filter;
  filter.setInputCloud(cloud);
  filter.setMeanK(50);
  filter.setStddevMulThresh(1.0);
  filter.filter(*cloud);
}

void NormalEstimation(
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
    pcl::PointCloud<PointT>::Ptr cloud,
    pcl::search::KdTree<PointT>::Ptr tree)
{
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud);
  ne.setKSearch(50);
  ne.compute(*cloud_normals);
}

std::vector<pcl::PointIndices> RegionGrowingRGB(
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
    pcl::PointCloud<PointT>::Ptr cloud,
    pcl::search::KdTree<PointT>::Ptr tree,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud)
{
  pcl::RegionGrowingRGB<PointT, pcl::Normal> reg;
  reg.setInputCloud(cloud);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(0.05);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(50);
  reg.setNumberOfNeighbours(30);
  reg.setInputNormals(cloud_normals);
  reg.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees.
  reg.setCurvatureThreshold(1.0);
  reg.setCurvatureTestFlag(false);
  reg.setNormalTestFlag(false);
  reg.setSmoothModeFlag(false);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  colored_cloud = reg.getColoredCloud();

  if (colored_cloud)
  {
    std::cerr << "Colored" << std::endl;
    colored_cloud->header = cloud->header;
    colored_cloud->sensor_origin_ = cloud->sensor_origin_;
    colored_cloud->sensor_orientation_ = cloud->sensor_orientation_;
  }
  return clusters;
}

bool isCylinder(
    pcl::PointCloud<PointT>::Ptr cloud,
    pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  if (cloud->size() == 0)
  {
    return false;
  }
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.05);
  seg.setRadiusLimits(0.09, 0.14);
  seg.setInputCloud(cloud);
  seg.setInputNormals(normals);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  seg.segment(*inliers, *coefficients);
  //std::cerr << "Inliers: " << inliers->indices.size() << std::endl;
  //std::cerr << "Cloud: " << cloud->size() << std::endl;
  float percentage = inliers->indices.size() / (float)cloud->size();
  //std::cerr << "Percentage: " << percentage << std::endl;
  return percentage > 0.9f;
}

void publishMarker(
    pcl::PointCloud<PointT>::Ptr cloud,
    geometry_msgs::TransformStamped tss)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);
  std::cerr << "centroid of the cylindrical component: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << " " << centroid[3] << std::endl;

  //Create a point in the "camera_rgb_optical_frame"
  geometry_msgs::PointStamped point_camera;
  geometry_msgs::PointStamped point_map;
  visualization_msgs::Marker marker;

  point_camera.header.frame_id = "camera_rgb_optical_frame";
  point_camera.header.stamp = ros::Time::now();

  point_map.header.frame_id = "map";
  point_map.header.stamp = ros::Time::now();

  point_camera.point.x = centroid[0];
  point_camera.point.y = centroid[1];
  point_camera.point.z = centroid[2];

  tf2::doTransform(point_camera, point_map, tss);

  std::cerr << "point_camera: " << point_camera.point.x << " " << point_camera.point.y << " " << point_camera.point.z << std::endl;

  std::cerr << "point_map: " << point_map.point.x << " " << point_map.point.y << " " << point_map.point.z << std::endl;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cylinder";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = point_map.point.x;
  marker.pose.position.y = point_map.point.y;
  marker.pose.position.z = point_map.point.z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;

  marker.lifetime = ros::Duration();

  pubm.publish(marker);
}

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
  geometry_msgs::TransformStamped tss;
  try{
     tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", ros::Time(0));
  }
  catch (tf2::ExtrapolationException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return;
  }

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
  pcl::PCLPointCloud2 voxel_filtered;

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  voxel_filter.setInputCloud(cloud_blob);
  voxel_filter.setLeafSize(0.02f, 0.02f, 0.02f);
  voxel_filter.filter(voxel_filtered);

  pcl::fromPCLPointCloud2(voxel_filtered, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size() << " data points." << std::endl;

  PassThroughFilter(cloud);
  StatisticalOutlierRemovalFilter(cloud);

  std::cerr << "PointCloud after filtering has: " << cloud->points.size() << " data points." << std::endl;

  pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  tree->setInputCloud(cloud);

  NormalEstimation(cloud_normals, cloud, tree);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;
  std::vector<pcl::PointIndices> clusters = RegionGrowingRGB(cloud_normals, cloud, tree, colored_cloud);

  if (colored_cloud)
  {
    std::cerr << clusters.size() << std::endl;
    for (int i = 0; i < clusters.size(); i++)
    {
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
      for (int k = 0; k < clusters[i].indices.size(); k++)
      {
        inliers->indices.push_back(clusters[i].indices[k]);
      }
      pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr cluster_normals(new pcl::PointCloud<pcl::Normal>);

      extract.setInputCloud(colored_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*cloud_cluster);

      extract_normals.setInputCloud(cloud_normals);
      extract_normals.setIndices(inliers);
      extract_normals.setNegative(false);
      extract_normals.filter(*cluster_normals);

      if (isCylinder(cloud_cluster, cluster_normals))
      {
        publishMarker(cloud_cluster, tss);

        pcl::PCLPointCloud2 out;
        pcl::toPCLPointCloud2(*cloud_cluster, out);
        puby.publish(out);
      }
    }
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");
  ros::NodeHandle nh;

  // For transforming between coordinate frames
  tf2_ros::TransformListener tf2_listener(tf2_buffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);

  pubm = nh.advertise<visualization_msgs::Marker>("detected_cylinder", 1);

  // Spin
  ros::spin();
}
