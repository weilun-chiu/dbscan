#pragma once

#include <vector>
#include <string>
#include <functional>
#include <numeric>
#include <deque>
#include <atomic>

#include "point.h"

class DBSCAN {
public:
    DBSCAN(double _eps, int _minPts) : eps(_eps), minPts(_minPts) {}
    void run(std::string filename);
    void run(std::vector<std::vector<double>>);
protected:
    const double eps;
    const int minPts;
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
            parent[i].store(-1, std::memory_order_relaxed);
        }
    }
    
    void setup(int n) {
        parent = std::vector<std::atomic<int>>(n);
        for (int i = 0; i < n; i++) {
            parent[i].store(-1, std::memory_order_relaxed);
        }
    }

    int find(int x) {
        while (x != parent[x].load(std::memory_order_relaxed)) {
            if (parent[x].load(std::memory_order_relaxed) == -1) {
                parent[x].store(x, std::memory_order_relaxed);
                return x;
            }
            x = parent[x].load(std::memory_order_relaxed);
        }
        return x;
    }
    
    void unite(int x, int y) {
        int root_x = find(x);
        int root_y = find(y);
        while (root_x != root_y) {
            if (root_x < root_y) {
                if (parent[root_y].compare_exchange_strong(root_y, root_x, std::memory_order_relaxed)) {
                    break;
                }
            } else {
                if (parent[root_x].compare_exchange_strong(root_x, root_y, std::memory_order_relaxed)) {
                    break;
                }
            }
            root_x = find(x);
            root_y = find(y);
        }
    }

    friend std::ostream& operator<<(std::ostream& os, const UnionFind& uf) {
        os << "[";
        for (int i = 0; i < uf.parent.size(); i++) {
            os << uf.parent[i].load(std::memory_order_relaxed);
            if (i != uf.parent.size() - 1) {
                os << ", ";
            }
        }
        os << "]";
        return os;
    }

private:
    std::vector<std::atomic<int>> parent;
};

class GridDBSCAN : public DBSCAN{
public:
    GridDBSCAN(double _eps, int _minPts, std::string _className);
protected:
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
    const std::string className;
    void assignPoints(std::vector<Point> const& points);
    void mark_ingrid_corecell();
    void mark_outgrid_corecell();
    virtual void expand() = 0;
    std::vector<int> getClusterResults(std::vector<Point> const&);
    std::vector<int> findNeighbor(int i);
    bool mark_outgrid_corecell_helper(int i);
    void _expand_helper(int i);
    void expand_helper(int i);
    std::vector<Point> preprocess(std::vector<Point> const& vp) override;
    std::vector<int> dbscan_algorithm(std::vector<Point> const& points) override;
};

class SerialGridDBSCAN : public GridDBSCAN{
public:
    SerialGridDBSCAN(double _eps, int _minPts) : GridDBSCAN(_eps, _minPts, "SerialGridDBSCAN") {}
protected:
    void expand() override;
};

class OMPGridDBSCAN : public GridDBSCAN{
public:
    OMPGridDBSCAN(double _eps, int _minPts);
protected:
    void expand() override;
};

class ConcurrencyGridDBSCAN : public GridDBSCAN{
public:
    ConcurrencyGridDBSCAN(double _eps, int _minPts);
protected:
    void expand() override;
    void expand_helper(int lo, int hi);
};

class ConcurrencyStealingGridDBSCAN : public GridDBSCAN{
public:
    ConcurrencyStealingGridDBSCAN(double _eps, int _minPts);
protected:
    void expand() override;
    void expand_helper(std::deque<int> *cells, int tid, std::deque<int> *neighbor, int nid);
};

class ConcurrencyStealingAVX2GridDBSCAN : public GridDBSCAN{
public:
    ConcurrencyStealingAVX2GridDBSCAN(double _eps, int _minPts);
protected:
    void expand() override;
    void expand_helper(std::deque<int> *cells, int tid, std::deque<int> *neighbor, int nid);
};