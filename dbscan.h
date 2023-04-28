#pragma once

#include <vector>
#include <string>
#include <functional>
#include <numeric>

#include "point.h"

class DBSCAN {
public:
    DBSCAN(double _eps, int _minPts) : eps(_eps), minPts(_minPts) {}
    void run(std::string filename);
    void run(std::vector<std::vector<double>>);
protected:
    double eps;
    int minPts;
    virtual std::vector<Point> preprocess(std::vector<Point> const& vp) = 0;
    virtual std::vector<int> dbscan_algorithm(std::vector<Point> const& points) = 0;
};

class NaiveDBSCAN : public DBSCAN{
public:
    NaiveDBSCAN(double _eps, int _minPts) : DBSCAN(_eps, _minPts) {}
protected:
    std::vector<Point> preprocess(std::vector<Point> const& vp) override {return vp;}
    std::vector<int> dbscan_algorithm(std::vector<Point> const& points) override;
};

class UnionFind {
public:
    UnionFind() {}
    UnionFind(int n) : parent(n) {
        for (int i = 0; i < n; i++) {
            parent[i] = i;
        }
    }
    
    void setup(const int& n) {
        parent.resize(n, 0);
        std::iota(parent.begin(), parent.end(), 0);
    }

    int find(int x) {
        while (x != parent[x]) {
            x = parent[x];
        }
        return x;
    }
    
    void unite(int x, int y) {
        int root_x = find(x);
        while (y != parent[y]) {
            int next = parent[y];
            parent[y] = root_x;
            y = next;
        }
    }

private:
    std::vector<int> parent;
};

class GridDBSCAN : public DBSCAN{
public:
    GridDBSCAN(double _eps, int _minPts);
private:
    int gridSize1D;
    double gridCellSize;
    std::vector<int> gridSize;
    std::vector<std::vector<Point>> grid;
    std::vector<bool> corecell;
    std::vector<int> corecell_set;
    std::vector<int> visited;
    std::vector<int> cluster;
    int clusterIdx;
    int npoints;
    UnionFind uf;
    void assignPoints(std::vector<Point> const& points);
    void mark_ingrid_corecell();
    void mark_outgrid_corecell();
    void expand();
    std::vector<int> getClusterResults();
    std::vector<int> findNeighbor(int i);
    bool mark_outgrid_corecell_helper(int i);
    void _expand_helper(int i);
    void expand_helper(int i);
protected:
    std::vector<Point> preprocess(std::vector<Point> const& vp) override;
    std::vector<int> dbscan_algorithm(std::vector<Point> const& points) override;
};