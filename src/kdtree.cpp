#include "kdtree.h"
#include "point.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "utils.h"


KDTree::KDTree(std::vector<Point> const& ps, int _leafsize) : root(std::make_unique<KDNode>()), leafsize(_leafsize) {
    buildTree(ps, root.get(), 0);
}

void KDTree::buildTree(std::vector<Point> const& points, KDNode * curr_node, size_t depth) {
    if (points.empty()) {
        return;
    }

    size_t num_points = points.size();

    // Base case: if number of points is less than or equal to leafsize, create a leaf node
    if (num_points <= leafsize) {
        for (auto const& p : points)
            curr_node->points.push_back(p.id);
        curr_node->isLeaf = true;
        return;
    }

    size_t axis = depth % Point::dimensionality;
    size_t median_index = num_points / 2;
    std::vector<Point> cpPoints(points);
    std::nth_element(cpPoints.begin(), cpPoints.begin() + median_index, cpPoints.end(), [&](const Point& lhs, const Point& rhs) { return lhs[axis] < rhs[axis]; });

    curr_node->pivot = points[median_index];
    curr_node->left = std::make_unique<KDNode>();
    curr_node->right = std::make_unique<KDNode>();
    
    buildTree(std::vector<Point>(cpPoints.begin(), cpPoints.begin() + median_index), curr_node->left.get(), depth + 1);
    buildTree(std::vector<Point>(cpPoints.begin() + median_index, cpPoints.end()), curr_node->right.get(), depth + 1);
}

std::vector<int> KDTree::search(const Point& target_point, double eps) {
    std::vector<int> results;
    searchHelper(root.get(), target_point, eps, results, 0);
    return results;
}

void KDTree::searchHelper(KDNode const * const curr_node, Point const& target_point, double eps, std::vector<int>& results, size_t depth) {
    if (curr_node == nullptr) {
        return;
    }

    if (curr_node->isLeaf) {
        results.insert(results.end(), curr_node->points.begin(), curr_node->points.end());
        return;
    }

    size_t axis = depth % Point::dimensionality;
    if (curr_node->left != nullptr && (target_point[axis] - eps <= curr_node->pivot[axis])) {
        searchHelper(curr_node->left.get(), target_point, eps, results, depth + 1);
    }
    if (curr_node->right != nullptr && (target_point[axis] + eps >= curr_node->pivot[axis])) {
        searchHelper(curr_node->right.get(), target_point, eps, results, depth + 1);
    }
}