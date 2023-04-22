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
        curr_node->points = ps;
        curr_node->isLeaf = true;
        // for (auto &p : curr_node->points)
        //     std::cout << p << std::endl;
        // std::cout << "num_points in leaf: " << num_points << std::endl;
        return;
    }

    size_t axis = depth % Point::dimensionality;
    size_t median_index = num_points / 2;
    std::nth_element(ps.begin(), ps.begin() + median_index, ps.end(), [&](const Point& lhs, const Point& rhs) { return lhs[axis] < rhs[axis]; });
    // for (int i=0; i < ps.size(); i++) {
    //     if (i < median_index && ps[i][axis] > ps[median_index][axis])
    //         std::cout << "Exception1" << std::endl;
    //     if (i > median_index && ps[i][axis] < ps[median_index][axis])
    //         std::cout << "Exception2" << std::endl;
    // }

    curr_node->pivot = ps[median_index];
    curr_node->left = std::make_shared<KDNode>();
    curr_node->right = std::make_shared<KDNode>();
    
    buildTreeHelper(std::vector<Point>(ps.begin(), ps.begin() + median_index), curr_node->left, depth + 1);
    buildTreeHelper(std::vector<Point>(ps.begin() + median_index, ps.end()), curr_node->right, depth + 1);
}

std::vector<Point> KDTree::search(const Point& target_point, const double eps) {
    std::vector<Point> points;
    // std::cout << "start_search " << target_point <<std::endl;
    searchHelper(root.get(), target_point, eps, points, 0);
    return points;
}

void KDTree::searchHelper(const KDNode* curr_node, const Point& target_point, const double eps, std::vector<Point>& points, const size_t depth) {
    if (curr_node == nullptr) {
        return;
    }

    if (curr_node->isLeaf) {
        if (curr_node->points.size() > 0)
            for (auto &p : curr_node->points) {   
                if (dist(p, target_point) <= eps)
                    points.push_back(p);
            }
        return;
    }

    size_t axis = depth % Point::dimensionality;
    if (curr_node->left != nullptr) {

    // && (target_point[axis] - eps <= curr_node->pivot[axis])) {
        searchHelper(curr_node->left.get(), target_point, eps, points, depth + 1);
    }
    if (curr_node->right != nullptr) {
        //  && (target_point[axis] + eps >= curr_node->pivot[axis])) {
        searchHelper(curr_node->right.get(), target_point, eps, points, depth + 1);
    }
}
