#include "../include/occupancy_map.h"
#include "../include/object_detector.h"
#include <cv_bridge/cv_bridge.h>
#include "../include/trajectory_planer.h"

OccupancyMap::OccupancyMap() : Node("om"){
    subPC = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/rgb/points", 5, std::bind(&OccupancyMap::pc_callback, this, std::placeholders::_1));
    pubOccupancyMapVis = this->create_publisher<sensor_msgs::msg::Image>("/occupancyMap/vis", 5);
}

void OccupancyMap::create_occupancy_map(const sensor_msgs::msg::PointCloud2::SharedPtr pc, cv::Mat& occupancyMap){
    int cols = static_cast<int>(mapWidth / resolution);
    int rows = static_cast<int>(mapHeight / resolution);

    occupancyMap = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(0));

    for (int i = 0; i < pc->data.size(); i += 4){
        if(pc->data[i + 2] < this->groundFilter){
            continue;
        }
        int col = static_cast<int>(((pc->data[i] + mapWidth) / 2) / resolution);
        int row = static_cast<int>(((pc->data[i+1] + mapWidth) / 2) / resolution);

        if (col >= 0 && col < cols && row >= 0 && row < rows){
            occupancyMap.at<int>(row, col) = 1;
        }
    }
}

void OccupancyMap::pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    cv::Mat om;
    create_occupancy_map(msg, om);
    publish_occupancyMapVis(om);
}

void OccupancyMap::publish_occupancyMapVis(cv::Mat& occupancyMap){
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "8UC1", occupancyMap).toImageMsg();
    pubOccupancyMapVis->publish(*msg.get());
}

void OccupancyMap::publish_occupancyMap(cv::Mat& occupancyMap){

}

std::vector<Point> astar_pathfinding(std::vector<std::vector<int>>& grid, Point start, Point goal){
    int rows = grid.size(), cols = grid[0].size();
    std::vector<Point> moves = {{-1, 0}, {1,0}, {0,1}, {0,-1}};

    std::vector<Point> path;

    std::priority_queue<NodeS, std::vector<NodeS>, std::greater<NodeS>> openSet;
    std::unordered_map<Point, Point, PointHash> cameFrom;
    std::unordered_map<Point, int, PointHash> gScore;

    openSet.push({heuristic(start, goal), 0, start});
    gScore[start] = 0;

    while (!openSet.empty()){
        NodeS current = openSet.top();
        openSet.pop();

        if (current.position == goal){
            
            for (Point at = goal; cameFrom.find(at) != cameFrom.end(); at = cameFrom[at]){
                path.push_back(at);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (auto move : moves){
            Point neighbor = {current.position.first + move.first, current.position.second + move.second};

            if (neighbor.first >= 0 && neighbor.first < rows && neighbor.second >= 0 && neighbor.second < cols && grid[neighbor.first][neighbor.second] == 0){
                int newGscore = gScore[current.position] + 1;

                if (!gScore.count(neighbor) || newGscore < gScore[neighbor]){
                    gScore[neighbor] = newGscore;
                    int fScore = newGscore + heuristic(neighbor, goal);
                    openSet.push({fScore, newGscore, neighbor});
                    cameFrom[neighbor] = current.position;
                }
            }
        }
    }
    return path;
}




int main(int argc, char *argv[]) {

    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 1},
        {0, 0, 0, 0, 0}
    };
    Point start = {0, 0}, goal = {4, 4};

    TrajectoryPlaner tp;


    std::vector<Point> path = astar_pathfinding(grid, start, goal);
    std::vector<PointHermite> pathHermite;
    std::vector<double> times;
    for (int i = 0; i < path.size(); i++){
        times.push_back(static_cast<double>(i));
        PointHermite p;
        p.x = static_cast<double>(path[i].first) + 0.5;
        p.y = static_cast<double>(path[i].second) + 0.5;
        pathHermite.push_back(p);
    }

    int resolution = 10;

    tp.generate_trajectory(pathHermite, times, resolution);


    
    for (auto p : path) {
        std::cout << "(" << p.first << ", " << p.second << ") -> ";
    }
    std::cout << "Goal" << std::endl;
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetector>());
    rclcpp::shutdown();
    return 0;
}