#include "Process.h"
#include <unordered_map>
#include <Eigen/Dense>
#include <iostream>

std::vector<Point> Process::removeNaN(const std::vector<Point>& points) {
    std::vector<Point> filtered;
    for (const auto& point : points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<Point> Process::removeOutliers(const std::vector<Point>& points, int threshold) {
    std::vector<Point> filtered;
    std::unordered_map<int, int> histogram;
    for (const auto& point : points) {
        int key = static_cast<int>(point.x * 100) + static_cast<int>(point.y * 100) + static_cast<int>(point.z * 100);
        histogram[key]++;
    }
    for (const auto& point : points) {
        int key = static_cast<int>(point.x * 100) + static_cast<int>(point.y * 100) + static_cast<int>(point.z * 100);
        if (histogram[key] < threshold) {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<Point> Process::downsample(const std::vector<Point>& points, float leafSize) {
    std::vector<Point> downsampled;
    std::unordered_map<Eigen::Vector3i, std::vector<Point>, Vector3iHash> grid;

    for (const auto& point : points) {
        Eigen::Vector3i voxelIdx(
            static_cast<int>(std::floor(point.x / leafSize)),
            static_cast<int>(std::floor(point.y / leafSize)),
            static_cast<int>(std::floor(point.z / leafSize))
        );
        grid[voxelIdx].push_back(point);
    }

    for (const auto& entry : grid) {
        const auto& voxelPoints = entry.second;
        if (!voxelPoints.empty()) {
            Eigen::Vector3f centroid(0, 0, 0);
            for (const auto& point : voxelPoints) {
                centroid += Eigen::Vector3f(point.x, point.y, point.z);
            }
            centroid /= static_cast<float>(voxelPoints.size());
            downsampled.emplace_back(Point{centroid.x(), centroid.y(), centroid.z()});
        }
    }

    return downsampled;
}
