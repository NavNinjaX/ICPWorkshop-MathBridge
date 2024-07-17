#ifndef PROCESS_H
#define PROCESS_H

#include <vector>
#include "Point.h"
#include <Eigen/Dense>

class Process {
public:
    static std::vector<Point> removeNaN(const std::vector<Point>& points);
    static std::vector<Point> removeOutliers(const std::vector<Point>& points, int threshold);
    static std::vector<Point> downsample(const std::vector<Point>& points, float leafSize);

private:
    struct Vector3iHash {
        std::size_t operator()(const Eigen::Vector3i& v) const {
            return std::hash<int>()(v.x()) ^ std::hash<int>()(v.y()) ^ std::hash<int>()(v.z());
        }
    };
};

#endif // PROCESS_H
