
#pragma once

#include <vector>
#include <memory>
#include <set>
#include "point.h"
#include "kdnode.h"

class KDTree {
public:
    // CTORs
    KDTree(std::vector<Point> const& points, int _leafsize=30);
    // rm CPYTORs, assignment op
    KDTree(KDTree const& other)=delete;
    KDTree& operator=(KDTree const& other)=delete;
    // API
    std::vector<int> search(const Point& target_point, double distance);
private:

    std::unique_ptr<KDNode> root;
    size_t leafsize;
    void buildTree(std::vector<Point> const& points, KDNode * curr_node, size_t depth);
    void searchHelper(KDNode const * const curr_node, Point const& target_point, double eps, std::vector<int>& results, size_t depth);
};