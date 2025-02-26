#pragma once 

// ROS
#include <rclcpp/rclcpp.hpp>


// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

// MESSAGES
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class ObjectDetector : public rclcpp::Node{
public:
    bool _debug_ransac = true;
    bool _debug_voxel = true;
    bool _debug_clustering = true;
    std::string _name = "ObjectDetector";
    double _minPercentage = 0.1;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPCplane, pubPCobj, pubPCcolor, pubPCvox;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC;
    ObjectDetector();
    void ransac_ground_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void voxel_grid_filter(pcl::PCLPointCloud2::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    double angle_tilt(pcl::ModelCoefficients::Ptr coefficients);
    void publish_pointcloud(pcl::PointCloud<pcl::PointXYZ> pc, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void publish_pointcloud_rgb(pcl::PointCloud<pcl::PointXYZRGB> pc, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void plane_segmentation(const sensor_msgs::msg::PointCloud2::SharedPtr in);
    double distance_point2plane(pcl::PointXYZ point, pcl::ModelCoefficients::Ptr coefficients);
private:
};