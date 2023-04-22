#include "kdtree.h"
#include "point.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include "utils.h"


KDTree::KDTree(std::vector<Point> ps, int _leafsize) : root(std::make_shared<KDNode>()), leafsize(_leafsize) {
    buildTreeHelper(ps, root, 0);
}

void KDTree::buildTreeHelper(std::vector<Point> ps, std::shared_ptr<KDNode>& curr_node, const size_t depth) {
    if (ps.empty()) {
        return;
    }

    size_t num_points = ps.size();

    // Base case: if number of points is less than or equal to leafsize, create a leaf node
    if (num_points <= leafsize) {
        for (auto &p : ps)
            curr_node->points.push_back(p.id);
        curr_node->isLeaf = true;
        return;
    }

    size_t axis = depth % Point::dimensionality;
    size_t median_index = num_points / 2;
    std::nth_element(ps.begin(), ps.begin() + median_index, ps.end(), [&](const Point& lhs, const Point& rhs) { return lhs[axis] < rhs[axis]; });

    curr_node->pivot = ps[median_index];
    curr_node->left = std::make_shared<KDNode>();
    curr_node->right = std::make_shared<KDNode>();
    
    buildTreeHelper(std::vector<Point>(ps.begin(), ps.begin() + median_index), curr_node->left, depth + 1);
    buildTreeHelper(std::vector<Point>(ps.begin() + median_index, ps.end()), curr_node->right, depth + 1);
}

std::vector<int> KDTree::search(const Point& target_point, const double eps) {
    std::vector<int> points;
    searchHelper(root.get(), target_point, eps, points, 0);
    return points;
}

void KDTree::searchHelper(const KDNode* curr_node, const Point& target_point, const double eps, std::vector<int>& points, const size_t depth) {
    if (curr_node == nullptr) {
        return;
    }

    if (curr_node->isLeaf) {
        points.insert(points.end(), curr_node->points.begin(), curr_node->points.end());
        return;
    }

    size_t axis = depth % Point::dimensionality;
    if (curr_node->left != nullptr && (target_point[axis] - eps <= curr_node->pivot[axis])) {
        searchHelper(curr_node->left.get(), target_point, eps, points, depth + 1);
    }
    if (curr_node->right != nullptr && (target_point[axis] + eps >= curr_node->pivot[axis])) {
        searchHelper(curr_node->right.get(), target_point, eps, points, depth + 1);
    }
}
