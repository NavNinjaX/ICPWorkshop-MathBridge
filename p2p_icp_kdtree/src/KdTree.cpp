#include "KdTree.h"
#include <cmath>
#include <algorithm>

KdTree::KdTree(const std::vector<Point>& points) {
    std::vector<Point> pts = points;
    root = build(pts, 0);
}

KdTree::~KdTree() {
    destroy(root);
}

KdNode* KdTree::build(std::vector<Point>& points, int depth) {
    if (points.empty()) return nullptr;

    int axis = depth % 3;
    std::sort(points.begin(), points.end(), [axis](const Point& a, const Point& b) {
        return axis == 0 ? a.x < b.x : (axis == 1 ? a.y < b.y : a.z < b.z);
    });

    size_t medianIdx = points.size() / 2;
    KdNode* node = new KdNode(points[medianIdx]);

    std::vector<Point> leftPoints(points.begin(), points.begin() + medianIdx);
    std::vector<Point> rightPoints(points.begin() + medianIdx + 1, points.end());

    node->left = build(leftPoints, depth + 1);
    node->right = build(rightPoints, depth + 1);

    return node;
}

void KdTree::destroy(KdNode* node) {
    if (node) {
        destroy(node->left);
        destroy(node->right);
        delete node;
    }
}

Point KdTree::findNearestNeighbor(const Point& target) const {
    Point bestPoint;
    float bestDist = std::numeric_limits<float>::max();
    nearestNeighbor(root, target, 0, bestPoint, bestDist);
    return bestPoint;
}

void KdTree::nearestNeighbor(KdNode* node, const Point& target, int depth, Point& bestPoint, float& bestDist) const {
    if (!node) return;

    float dist = std::sqrt(std::pow(node->point.x - target.x, 2) +
                           std::pow(node->point.y - target.y, 2) +
                           std::pow(node->point.z - target.z, 2));

    if (dist < bestDist) {
        bestDist = dist;
        bestPoint = node->point;
    }

    int axis = depth % 3;
    KdNode* nextNode = (axis == 0 ? (target.x < node->point.x ? node->left : node->right) :
                       (axis == 1 ? (target.y < node->point.y ? node->left : node->right) :
                                    (target.z < node->point.z ? node->left : node->right)));
    KdNode* otherNode = (nextNode == node->left ? node->right : node->left);

    nearestNeighbor(nextNode, target, depth + 1, bestPoint, bestDist);

    float axisDist = axis == 0 ? std::abs(node->point.x - target.x) :
                      (axis == 1 ? std::abs(node->point.y - target.y) :
                                   std::abs(node->point.z - target.z));

    if (axisDist < bestDist) {
        nearestNeighbor(otherNode, target, depth + 1, bestPoint, bestDist);
    }
}
