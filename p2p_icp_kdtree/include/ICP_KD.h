#ifndef ICP_KD_H
#define ICP_KD_H

#include <chrono>
#include <iostream>
#include <fstream>

#include <vector>
#include "Point.h"
#include "KdTree.h"
#include <Eigen/Dense>


class ICP {
public:
    ICP(const std::vector<Point>& sourcePoints, const std::vector<Point>& targetPoints);
    void align(int iterations);
    std::vector<Point> getTransformedSourcePoints() const;
    Eigen::Matrix4d getFinalTransformation() const;

    Eigen::Matrix3f es_rotation;
    Eigen::Vector3f es_translation;


private:
    std::vector<Point> sourcePoints;
    std::vector<Point> targetPoints;
    std::vector<Point> transformedSourcePoints;
    KdTree targetKdTree;

    std::vector<std::pair<Point, Point>> findCorrespondences();
    void estimatePose(const std::vector<std::pair<Point, Point>>& correspondences, Eigen::Matrix3f& R, Eigen::Vector3f& t);
};

#endif // ICP_KD_H
