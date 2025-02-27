#pragma once

#include <rclcpp/rclcpp.hpp>

struct PointHermite {
    double x, y;
    double vx, vy;
};

class TrajectoryPlaner  {
public:
    TrajectoryPlaner();
    double h00(double t) { return 2*t*t*t - 3*t*t + 1; }
    double h01(double t) { return -2*t*t*t + 3*t*t; }
    double h10(double t) { return t*t*t - 2*t*t + t; }
    double h11(double t) { return t*t*t - t*t; }
    std::vector<PointHermite> compute_velocities(std::vector<PointHermite>& points, std::vector<double>& times);
    PointHermite evaluate_hermite(double t, PointHermite& p0, PointHermite& p1, double t0, double t1);
    void generate_trajectory(std::vector<PointHermite>& waypoints, std::vector<double>& times, int resolution);

};