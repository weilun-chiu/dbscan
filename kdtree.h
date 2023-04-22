#pragma once

#include <vector>
#include <memory>
#include <set>
#include "point.h"
#include "kdnode.h"

class KDTree {
public:
    // CTORs
    KDTree(std::vector<Point> ps, int _leafsize=30);
    // rm CPYTORs, assignment op
    KDTree(KDTree const& other)=delete;
    KDTree& operator=(KDTree const& other)=delete;
    // API
    
    std::vector<Point> search(const Point& target_point, const double distance);
private:

    std::shared_ptr<KDNode> root;
    size_t leafsize;
    void buildTreeHelper(std::vector<Point> points, std::shared_ptr<KDNode>& curr_node, const size_t depth);
    void searchHelper(const KDNode* curr_node, const Point& target_point, const double distance, std::vector<Point>& points, const size_t depth);
};