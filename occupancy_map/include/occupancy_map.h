#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <sensor_msgs/msg/image.hpp>

#include <queue>
#include <unordered_map>
#include <cmath>

#include <opencv2/opencv.hpp>



struct Point {
    int first, second;
    bool operator==(const Point& other) const {
        return first == other.first && second == other.second;
    }
};

struct PointHash {
    std::size_t operator()(const Point& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};


struct NodeS {
    int f, g;
    Point position;
    bool operator>(const NodeS& other) const {
        return f > other.f;
    }
};

int heuristic(Point a, Point b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

class OccupancyMap : public rclcpp::Node{
public:
    std::vector<std::vector<int>> grid;
    float groundFilter = 10;
    float resolution = 20;
    float mapWidth = 100;
    float mapHeight = 100; 

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubOccupancyMapVis;



    OccupancyMap();
    std::vector<Point> astar_pathfinding(std::vector<std::vector<int>>& grid, Point start, Point goal);
    void create_occupancy_map(const sensor_msgs::msg::PointCloud2::SharedPtr pc, cv::Mat& occupancyMap);
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publish_occupancyMapVis(cv::Mat& occupancyMap);
    void publish_occupancyMap(cv::Mat& occupancyMap);
};