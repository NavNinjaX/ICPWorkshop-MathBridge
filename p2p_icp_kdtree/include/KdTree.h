#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <limits>
#include "Point.h"

struct KdNode{
    Point point;
    KdNode* left;
    KdNode* right;

    KdNode(const Point& pt) : point(pt), left(nullptr), right(nullptr){}
};

class KdTree{
public: 
    KdTree(const std::vector<Point>& points);
    ~KdTree();

    Point findNearestNeighbor(const Point& target) const;

    private:
    KdNode* root;
    KdNode* build(std::vector<Point>& points, int depth);
    void destroy(KdNode* node);
    void nearestNeighbor(KdNode* node, const Point& target, int depth, Point& bestPoint, float& bestDist) const; 
};


#endif //KDTREE_H