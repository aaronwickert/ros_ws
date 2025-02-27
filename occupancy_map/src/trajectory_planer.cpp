#include "../include/trajectory_planer.h"

TrajectoryPlaner::TrajectoryPlaner(){
    
}

std::vector<PointHermite> TrajectoryPlaner::compute_velocities(std::vector<PointHermite>& points, std::vector<double>& times) {
    std::vector<PointHermite> result = points;
    int n = points.size();

    for (int i = 0; i < n; ++i) {
        if (i == 0) {  // Forward difference for first point
            result[i].vx = (points[i + 1].x - points[i].x) / (times[i + 1] - times[i]);
            result[i].vy = (points[i + 1].y - points[i].y) / (times[i + 1] - times[i]);
        } 
        else if (i == n - 1) {  // Backward difference for last point
            result[i].vx = (points[i].x - points[i - 1].x) / (times[i] - times[i - 1]);
            result[i].vy = (points[i].y - points[i - 1].y) / (times[i] - times[i - 1]);
        } 
        else {  // Central difference for middle points
            result[i].vx = (points[i + 1].x - points[i - 1].x) / (times[i + 1] - times[i - 1]);
            result[i].vy = (points[i + 1].y - points[i - 1].y) / (times[i + 1] - times[i - 1]);
        }
    }

    return result;
}



void TrajectoryPlaner::generate_trajectory(std::vector<PointHermite>& waypoints, std::vector<double>& times, int resolution) {
    std::vector<PointHermite> points = compute_velocities(waypoints, times);

    for (size_t i = 0; i < points.size() - 1; ++i) {
        double t0 = times[i], t1 = times[i + 1];

        for (int j = 0; j <= resolution; ++j) {
            double t = t0 + j * (t1 - t0) / resolution;
            PointHermite interpolated = evaluate_hermite(t, points[i], points[i + 1], t0, t1);
            std::cout << "t=" << t << " -> (" << interpolated.x << ", " << interpolated.y << ")\n";
        }
    }
}

PointHermite TrajectoryPlaner::evaluate_hermite(double t, PointHermite& p0, PointHermite& p1, double t0, double t1) {
    double dt = t1 - t0;
    double s = (t - t0) / dt;  // Normalize t to [0,1]

    PointHermite result;
    result.x = h00(s) * p0.x + h01(s) * p1.x + h10(s) * p0.vx * dt + h11(s) * p1.vx * dt;
    result.y = h00(s) * p0.y + h01(s) * p1.y + h10(s) * p0.vy * dt + h11(s) * p1.vy * dt;

    return result;
}