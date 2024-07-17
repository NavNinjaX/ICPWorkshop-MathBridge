#include "ICP_KD.h"

ICP::ICP(const std::vector<Point>& sourcePoints, const std::vector<Point>& targetPoints)
    : sourcePoints(sourcePoints), targetPoints(targetPoints), targetKdTree(targetPoints) {}

void ICP::align(int iterations) {
    transformedSourcePoints = sourcePoints;
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = Eigen::Vector3f::Zero();

    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<std::pair<Point, Point>> correspondences = findCorrespondences();

        estimatePose(correspondences, R, t);

        for (Point& pt : transformedSourcePoints) {
            Eigen::Vector3f p(pt.x, pt.y, pt.z);
            Eigen::Vector3f p_transformed = R * p + t;
            pt.x = p_transformed(0);
            pt.y = p_transformed(1);
            pt.z = p_transformed(2);
        }
    }

    es_rotation = R;
    es_translation = t;
}

Eigen::Matrix4d ICP::getFinalTransformation() const {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = es_rotation.cast<double>();
    transformation.block<3, 1>(0, 3) = es_translation.cast<double>();
    return transformation;
}


std::vector<std::pair<Point, Point>> ICP::findCorrespondences() {
    std::vector<std::pair<Point, Point>> correspondences;

    for (const Point& sourcePoint : transformedSourcePoints) {
        Point nearestTargetPoint = targetKdTree.findNearestNeighbor(sourcePoint);
        correspondences.emplace_back(sourcePoint, nearestTargetPoint);
    }

    return correspondences;
}

void ICP::estimatePose(const std::vector<std::pair<Point, Point>>& correspondences, Eigen::Matrix3f& R, Eigen::Vector3f& t) {
    Eigen::Vector3f sourceCentroid(0, 0, 0);
    Eigen::Vector3f targetCentroid(0, 0, 0);

    for (const auto& correspondence : correspondences) {
        sourceCentroid += Eigen::Vector3f(correspondence.first.x, correspondence.first.y, correspondence.first.z);
        targetCentroid += Eigen::Vector3f(correspondence.second.x, correspondence.second.y, correspondence.second.z);
    }

    sourceCentroid /= correspondences.size();
    targetCentroid /= correspondences.size();

    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (const auto& correspondence : correspondences) {
        Eigen::Vector3f src(correspondence.first.x, correspondence.first.y, correspondence.first.z);
        Eigen::Vector3f tgt(correspondence.second.x, correspondence.second.y, correspondence.second.z);
        src -= sourceCentroid;
        tgt -= targetCentroid;
        H += src * tgt.transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    R = V * U.transpose();
    t = targetCentroid - R * sourceCentroid;
}

std::vector<Point> ICP::getTransformedSourcePoints() const {
    return transformedSourcePoints;
}
