#include "PLYReader.h"
#include "ICP_KD.h"
#include "Process.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

Eigen::Matrix4d createTransformationMatrix(double tx, double ty, double tz, double qx, double qy, double qz, double qw) {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation(0, 3) = tx;
    transformation(1, 3) = ty;
    transformation(2, 3) = tz;
    Eigen::Quaterniond q(qw, qx, qy, qz);
    transformation.block<3, 3>(0, 0) = q.toRotationMatrix();
    return transformation;
}

void calculatePoseError(const Eigen::Matrix4d& estimatedPose, const Eigen::Matrix4d& truePose) {
    Eigen::Vector3d estimatedTranslation = estimatedPose.block<3, 1>(0, 3);
    Eigen::Vector3d trueTranslation = truePose.block<3, 1>(0, 3);
    double translationError = (estimatedTranslation - trueTranslation).norm();

    Eigen::Matrix3d estimatedRotation = estimatedPose.block<3, 3>(0, 0);
    Eigen::Matrix3d trueRotation = truePose.block<3, 3>(0, 0);
    Eigen::Matrix3d rotationDifference = estimatedRotation * trueRotation.transpose();
    Eigen::AngleAxisd rotationError(rotationDifference);

    std::cout << "Translation Error: " << translationError << std::endl;
    std::cout << "Rotation Error (radians): " << rotationError.angle() << std::endl;
}

#if 0
void savePointCloudToPLY(const std::string& filename, const std::vector<Point>& points, const std::string& color) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << points.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property uchar red" << std::endl;
    file << "property uchar green" << std::endl;
    file << "property uchar blue" << std::endl;
    file << "end_header" << std::endl;

    int r, g, b;
    if (color == "red") {
        r = 255; g = 0; b = 0;
    } else if (color == "green") {
        r = 0; g = 255; b = 0;
    } else {
        r = 255; g = 255; b = 255;
    }


    for (const auto& point : points) {
        file << point.x << " " << point.y << " " << point.z << " " << r << " " << g << " " << b << std::endl;
    }


    file.close();
    std::cout << "Saved point cloud to " << filename << std::endl;
}
#endif

int main() {
    std::vector<Point> sourcePoints = PLYReader::read("/home/**/Dataset/stanford-bunny/bunny/data/bun315.ply");
    std::vector<Point> targetPoints = PLYReader::read("/home/**/Dataset/stanford-bunny/bunny/data/bun000.ply");

    if (sourcePoints.empty() || targetPoints.empty()) {
        std::cerr << "Failed to load point clouds." << std::endl;
        return -1;
    }

    // 手动定义真实位姿
    Eigen::Matrix4d truePoseSource = createTransformationMatrix(0, 0, 0, 0, 0, 0, 1);
    Eigen::Matrix4d truePoseTarget = createTransformationMatrix(-0.00646017, -1.36122e-05, -0.0129064, 0.00449209, 0.38422, -0.00976512, 0.923179);

    // Preprocess point clouds
    sourcePoints = Process::removeNaN(sourcePoints);
    targetPoints = Process::removeNaN(targetPoints);

    std::cout << "Source points after removing NaN: " << sourcePoints.size() << std::endl;
    std::cout << "Target points after removing NaN: " << targetPoints.size() << std::endl;

    sourcePoints = Process::downsample(sourcePoints, 0.03f); // Set an appropriate grid size
    targetPoints = Process::downsample(targetPoints, 0.03f);

    std::cout << "Source points after downsampling: " << sourcePoints.size() << std::endl;
    std::cout << "Target points after downsampling: " << targetPoints.size() << std::endl;

#if 1
    ICP icp(sourcePoints, targetPoints);

    auto start_total = std::chrono::high_resolution_clock::now();
    icp.align(50);
    auto end_total = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> duration_total = end_total - start_total;
    std::cout << "Total ICP Time: " << duration_total.count() << "s" << std::endl;

    Eigen::Matrix4d estimatedPose = icp.getFinalTransformation();
    calculatePoseError(estimatedPose, truePoseTarget);

    std::vector<Point> transformedSourcePoints = icp.getTransformedSourcePoints();
#endif

#if 1
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const Point& pt : targetPoints) {
        pcl::PointXYZ point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        targetCloud->points.push_back(point);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedsourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const Point& pt : transformedSourcePoints) {
        pcl::PointXYZ point;
        point.x = pt.x;
        point.y = pt.y;
        point.z = pt.z;
        transformedsourceCloud->points.push_back(point);
    }

    pcl::visualization::PCLVisualizer viewer("ICP Result Viewer");

    viewer.addPointCloud<pcl::PointXYZ>(targetCloud, "Target Cloud");
    viewer.addPointCloud<pcl::PointXYZ>(transformedsourceCloud, "transformed Source Cloud");

    viewer.setBackgroundColor(0, 0, 0);

    viewer.setCameraPosition(-0.0172, -0.0936, -0.734, -0.0461723, 0.970603, -0.235889, 0.0124573);

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

#endif

    return 0;
}
