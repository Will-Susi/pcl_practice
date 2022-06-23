#include "ros/subscriber.h"
#include <nav_msgs/Odometry.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

std::string cloud_points;
std::string cloud_output;

float min_filter_limit;
float max_filter_limit;
std::string filter_field;

float distance_threshold;
float point_color_threshold;
float region_color_threshold;
int min_cluster_size;

ros::Publisher pub;

void cloud_call_back(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

  if (size > 0) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    /*
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(raw_cloud);
    sor.setLeafSize (0.05f, 0.05f, 0.05f);
    sor.filter(*transformed_cloud);
    */

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName(filter_field);
    pass.setFilterLimits(min_filter_limit, max_filter_limit);
    pass.filter(*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(raw_cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(distance_threshold);
    reg.setPointColorThreshold(point_color_threshold);
    reg.setRegionColorThreshold(region_color_threshold);
    reg.setMinClusterSize(min_cluster_size);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    transformed_cloud = reg.getColoredCloud();

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);

    output_msg.header.frame_id = cloud_msg->header.frame_id;
    output_msg.header.stamp = cloud_msg->header.stamp;

    pub.publish(output_msg);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_tutorial");
  ros::NodeHandle nh("~");

  nh.param<std::string>("cloud_points", cloud_points, "/camera/depth/points");
  nh.param<std::string>("cloud_output", cloud_output, "output");

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(
      "/camera/depth/points", 10, cloud_call_back);
  pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_output, 1);

  while (ros::ok()) {
    nh.param<float>("min_filter_limit", min_filter_limit, 0.0);
    nh.param<float>("max_filter_limit", max_filter_limit, 4.0);
    nh.param<std::string>("filter_field", filter_field, "z");

    nh.param<float>("distance_threshold", distance_threshold, 10);
    nh.param<float>("point_color_threshold", point_color_threshold, 6.0);
    nh.param<float>("region_color_threshold", region_color_threshold, 5.0);
    nh.param<int>("min_cluster_size", min_cluster_size, 600);

    ros::spinOnce();
  }
}