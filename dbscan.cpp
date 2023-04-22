#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <numeric>
#include <functional>
#include <chrono>
#include <queue>

#include "utils.h"
#include "point.h"
#include "kdtree.h"
#include "dbscan.h"



std::vector<int> NaiveDBSCAN::dbscan_algorithm(std::vector<Point> points) {
    std::cout << "Naive DBSCAN(eps="<<eps<<", minPts="<<minPts<<") on datasize: " << points.size() << " in " << Point::dimensionality<<"-dimension space" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<int> visited(points.size(), 0);
    std::vector<int> cluster(points.size(), -1);
    int clusterIdx = 0;

    for (int i = 0; i < points.size(); i++) {
        if (visited[i] == 1) continue;

        visited[i] = 1;
        std::vector<int> neighborIdxs = rangeQuery(points, i, eps);
        if (neighborIdxs.size() >= minPts) {
            cluster[i] = clusterIdx;
            points[i].cluster_label = clusterIdx;
            for (int j = 0; j < neighborIdxs.size(); j++) {
                int idx = neighborIdxs[j];
                if (visited[idx] == 0) {
                    visited[idx] = 1;
                    std::vector<int> subNeighborIdxs = rangeQuery(points, idx, eps);
                    if (subNeighborIdxs.size() >= minPts) {
                        neighborIdxs.insert(neighborIdxs.end(), subNeighborIdxs.begin(), subNeighborIdxs.end());
                    }
                }
                if (points[idx].cluster_label == -1) {
                    cluster[idx] = clusterIdx;
                    points[idx].cluster_label = clusterIdx;
                }
            }
            clusterIdx++;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " seconds." << std::endl;
    return cluster;
}

std::vector<int> getGridSize(const std::vector<double>& max_values, const std::vector<double>& min_values, double gridCellSize) {
    std::vector<int> gridSize(max_values.size());
    for (int i = 0; i < max_values.size(); i++) {
        gridSize[i] = static_cast<int>(std::ceil((max_values[i] - min_values[i]) / gridCellSize));
    }
    return gridSize;
}

int kDTo1DIdx(const std::vector<int>& index, const std::vector<int>& gridSize) {
    int k = index.size();
    int Idx1D = index[0];
    int stride = 1;
    for (int i = 1; i < k; i++) {
        stride *= gridSize[i-1];
        Idx1D += stride * index[i];
    }
    return Idx1D;
}

std::vector<int> oneDToKDIdx(int index, const std::vector<int>& gridSize) {
    std::vector<int> kDIdx(gridSize.size(), 0);
    int k = gridSize.size();
    for (int i = k - 1; i >= 0; i--) {
        kDIdx[i] = index % gridSize[i];
        index /= gridSize[i];
    }
    return kDIdx;
}

std::vector<int> getNeighborIndices(const std::vector<int>& index, const std::vector<int>& gridSize) {
    std::vector<int> res{};
    if (Point::dimensionality == 1) {
        res = {index[0]-1, index[0]+1};
    } else if (Point::dimensionality == 2) {
        for (int i = std::max(0,index[0]-2); i < std::min(index[0]+3, gridSize[0]); i++) {
            for (int j = std::max(0,index[1]-2); j < std::min(index[1]+3, gridSize[0]); j++) {
                if ((abs(i)+abs(j)) == 4) continue;
                if ((i == 0) && (j == 0)) continue;
                res.push_back((i)*gridSize[0]+(j));
            }
        }
    }
    return res;
}

bool isConnect(std::vector<Point> lhs, std::vector<Point> rhs, double eps) {
    for (const auto& lhsp: lhs)
        for(const auto& rhsp: rhs)
            if (dist(lhsp, rhsp) <= eps)
                return true;
    return false;
}

int getConnectCount(Point lhsp, std::vector<Point> rhs, double eps) {
    int numConn{0};
    for(const auto& rhsp: rhs)
        if (dist(lhsp, rhsp) <= eps)
            numConn++;
    return numConn;
}

GridDBSCAN::GridDBSCAN(double _eps, int _minPts)
    :   DBSCAN(_eps, _minPts) 
    {}

std::vector<Point> GridDBSCAN::preprocess(std::vector<Point> points) {
    gridCellSize = eps / sqrt(Point::dimensionality);
    auto [max_values, min_values] = calculateMinMaxValues(points);
    gridSize = getGridSize(max_values, min_values, gridCellSize);
    gridSize1D = std::accumulate(gridSize.begin(), gridSize.end(), 1, std::multiplies<int>());
    grid.resize(gridSize1D);
    corecell.resize(gridSize1D, false);
    visited.resize(gridSize1D, 0);
    cluster.resize(gridSize1D, -1);
    clusterIdx = 0;
    npoints = points.size();
    return points;
}
void GridDBSCAN::assignPoints(std::vector<Point> points) {
    for (const auto& p : points) {
        std::vector<int> index(Point::dimensionality);
        double _gridCellSize = gridCellSize;
        std::transform(p.begin(), p.end(), index.begin(), [_gridCellSize](double x) {
            return static_cast<int>(x / _gridCellSize);
        });
        int GridIndex1D(kDTo1DIdx(index, gridSize));
        grid[GridIndex1D].push_back(p);
    }
}
void GridDBSCAN::mark_ingrid_corecell() {
    for (int i=0; i<gridSize1D; i++) {
        if (grid[i].size() >= minPts) {
            corecell[i] = true;
        } 
    }
}
void GridDBSCAN::mark_outgrid_corecell() {
    for (int i=0; i<gridSize1D; i++) {
        if (corecell[i] == true) continue;
        if (grid[i].size() == 0) continue;
        if (grid[i].size() > 0) 
            corecell[i] = mark_outgrid_corecell_helper(i);
    }
}
void GridDBSCAN::expand() {
    for (int i = 0; i < gridSize1D; i++) {
        if (visited[i] == true) continue;
        visited[i] = true;
        if (corecell[i] == false) continue;
        cluster[i] = clusterIdx;
        expand_helper(i);
        clusterIdx++;
    }
}
std::vector<int> GridDBSCAN::getClusterResults () {
    std::vector<int> pointsCluster(npoints, -1);
    for (int i=0; i<gridSize1D; i++) {
        for (const auto& g: grid[i]) {
            pointsCluster[g.id] = cluster[i];
        }
    }
    return pointsCluster;
}
std::vector<int> GridDBSCAN::findNeighbor(int i) {
    std::vector<int> kDIndex(oneDToKDIdx(i, gridSize));
    std::vector<int> neighborsIn1D(getNeighborIndices(kDIndex, gridSize));
    return neighborsIn1D;
}
bool GridDBSCAN::mark_outgrid_corecell_helper(int i) {
    std::vector<int> neighbors=findNeighbor(i);
    for (const auto& g: grid[i]) {
        int numConn{0};
        for (const auto& nid: neighbors) {
            numConn += getConnectCount(g, grid[nid], eps);
            if (numConn >= minPts) {
                return true;
            }
        }
    }
    return false;
}
void GridDBSCAN::expand_helper(int i) {
    std::queue<int> q;
    q.push(i);
    while (!q.empty()) {
        int node = q.front();
        q.pop();
        std::vector<int> neighbors=findNeighbor(node);
        for (auto& ni:neighbors) {
            if (visited[ni] == true) continue;
            if (isConnect(grid[i], grid[ni], eps)) {
                visited[ni] = true;
                cluster[ni] = clusterIdx;
                if (corecell[ni]==true)
                    q.push(ni);
            }
        }
    }
}

std::vector<int> GridDBSCAN::dbscan_algorithm(std::vector<Point> points) {
    if (Point::dimensionality > 2) {
        std::cout << "Skip. Only support 2 dimension grid DBSCAN in this project." << std::endl;
        return std::vector<int>(points.size(), -1);
    }

    std::cout << "Grid DBSCAN(eps="<<eps<<", minPts="<<minPts<<") on datasize: " << points.size() << " in " << Point::dimensionality<<"-dimension space" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();

    // Assign points to grid
    assignPoints(points);
    // Mark core cell
    mark_ingrid_corecell();
    mark_outgrid_corecell();
    // Expand clustering
    expand();

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() << " seconds." << std::endl;
    
    std::vector<int> pointsCluster = getClusterResults();
    return pointsCluster;
}

std::vector<int> kdtree_dbscan(std::vector<Point>& points, double eps, int minPts) {
    std::vector<int> visited(points.size(), 0);
    std::vector<int> cluster(points.size(), -1);
    int clusterIdx = 0;

    KDTree kdtree(points);

    for (int i = 0; i < points.size(); i++) {
        if (visited[i]) continue;
        visited[points[i].id] = true;

        std::vector<int> neighborIdxs = kdtree.search(points[i], eps);
        auto it = neighborIdxs.begin();
        while (it != neighborIdxs.end()) {
            auto p(points[*it]);
            if (dist(points[i], p) > eps) {
                neighborIdxs.erase(it);
            } else {
                it++;
            }
        }

        if (neighborIdxs.size() < minPts) {
            cluster[i] = -1;
        } else {
            cluster[i] = clusterIdx;
            for (auto & neighbor : neighborIdxs) {
                if (visited[neighbor] == false) {
                    visited[neighbor] = true;
                    std::vector<int> subNeighborIdxs = kdtree.search(points[neighbor], eps);
                    if (subNeighborIdxs.size() >= minPts) {
                        for (auto &sp : subNeighborIdxs) {
                            if (visited[sp] == true)
                                continue;
                            visited[sp] = true;
                            neighborIdxs.push_back(sp);
                        }
                    }
                }
                if (cluster[neighbor] == -1) {
                    cluster[neighbor] = clusterIdx;
                }
            }
            clusterIdx++;
        }
    }
    return cluster;
}

void print_clusters(const std::vector<Point>& points, const std::vector<int>& cluster) {
    int clusterCount = *std::max_element(cluster.begin(), cluster.end()) + 1;
    std::cout << "Clusters: " << clusterCount << std::endl;
    for (int i = 0; i < points.size(); i++) {
        std::cout << "(";
        for (int j = 0; j < points[i].coords.size(); j++) {
            std::cout << points[i].coords[j];
            if (j < points[i].coords.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "): " << cluster[i] << std::endl;
    }
}

void DBSCAN::run(std::string filename) {
    Point::resetDimension();
    auto points(preprocess(normalize(parseDataset(filename))));
    auto cluster(dbscan_algorithm(points));
    // print_clusters(points, cluster);
}

void DBSCAN::run(std::vector<std::vector<double>> data) {
    Point::resetDimension();
    auto points(preprocess(normalize(parseRandomGeneratedData(data))));
    auto cluster(dbscan_algorithm(points));
    // print_clusters(points, cluster);
}