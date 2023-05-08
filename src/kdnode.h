#pragma once

#include <memory>
#include "point.h"

class KDNode {
public:
    typedef std::shared_ptr<KDNode> KDNodeptr;
    Point pivot;
    KDNodeptr left;
    KDNodeptr right;
    bool isLeaf;
    std::vector<int> points;

    // CTORs
    KDNode() : pivot(Point()), left(nullptr), right(nullptr), isLeaf(false), points{std::vector<int>()} {}

    // rm CPCTOR, assignment op
    KDNode(KDNode const& other)=delete;
    KDNode& operator=(KDNode const& other)=delete;

};