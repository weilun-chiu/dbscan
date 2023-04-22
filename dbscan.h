#pragma once

#include <vector>
#include <string>
#include <functional>
#include "point.h"

class DBSCAN {
public:
    DBSCAN(double _eps, int _minPts) : eps(_eps), minPts(_minPts) {}
    void run(std::string filename);
    void run(std::vector<std::vector<double>>);
protected:
    double eps;
    int minPts;
    virtual std::vector<Point> preprocess(std::vector<Point> vp) = 0;
    virtual std::vector<int> dbscan_algorithm(std::vector<Point> points) = 0;
};

class NaiveDBSCAN : public DBSCAN{
public:
    NaiveDBSCAN(double _eps, int _minPts) : DBSCAN(_eps, _minPts) {}
protected:
    std::vector<Point> preprocess(std::vector<Point> vp) override {return vp;}
    std::vector<int> dbscan_algorithm(std::vector<Point> points) override;
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
    std::vector<int> visited;
    std::vector<int> cluster;
    int clusterIdx;
    int npoints;
    void assignPoints(std::vector<Point> points);
    void mark_ingrid_corecell();
    void mark_outgrid_corecell();
    void expand();
    std::vector<int> getClusterResults();
    std::vector<int> findNeighbor(int i);
    bool mark_outgrid_corecell_helper(int i);
    void expand_helper(int i);
protected:
    std::vector<Point> preprocess(std::vector<Point> vp) override;
    std::vector<int> dbscan_algorithm(std::vector<Point> points) override;
};