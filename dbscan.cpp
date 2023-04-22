#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include <iterator>
#include <numeric>
#include <set>
#include <functional>

#include "utils.h"
#include "point.h"
#include "kdtree.h"
#include "dbscan.h"

std::tuple<std::vector<double>, std::vector<double>> calculateMinMaxValues(const std::vector<Point>& points) {
    std::vector<double> max_values(Point::dimensionality, std::numeric_limits<double>::min());
    std::vector<double> min_values(Point::dimensionality, std::numeric_limits<double>::max());
    
    for (const auto& point : points) {
        for (int i = 0; i < Point::dimensionality; i++) {
            max_values[i] = std::max(max_values[i], point[i]);
            min_values[i] = std::min(min_values[i], point[i]);
        }
    }

    return std::tie(max_values, min_values);
}

std::vector<double> normalize_point(const Point& point, const std::vector<double>& max_values, const std::vector<double>& min_values) {
    std::vector<double> normalized_point(Point::dimensionality);
    for (int i = 0; i < Point::dimensionality; i++) {
        normalized_point[i] = (point[i] - min_values[i]) / (max_values[i] - min_values[i]);
    }
    return normalized_point;
}

std::vector<Point> normalize(const std::vector<Point>& points) {
    std::vector<Point> result = points;

    if (points.empty()) {
        return result;
    }

    // Find the maximum and minimum values for each dimension
    auto [max_values, min_values] = calculateMinMaxValues(points);

    // Normalize the data points
    for (int i = 0; i < points.size(); i++) {
        std::vector<double> normalized_point = normalize_point(points[i], max_values, min_values);
        result[i] = normalized_point;
    }

    return result;
}

std::vector<int> rangeQuery(std::vector<Point>& points, int p, double eps) {
    std::vector<int> neighbors;
    for (int i = 0; i < points.size(); i++) {
        if (dist(points[p], points[i]) <= eps) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
}

std::vector<int> naive_dbscan(std::vector<Point>& points, double eps, int minPts) {
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
    return cluster;
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

std::vector<Point> parseDataset(std::string filename) 
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        throw std::runtime_error("Error opening file");
    }

    std::vector<Point> vp{};
    std::string line{};
    int idx{0};
    while (std::getline(infile, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');

        std::stringstream ss(line);
        std::vector<double> values{std::istream_iterator<double>(ss), {}};
        if (values.empty()) {
            continue;
        }

        int label = static_cast<int>(values.back());
        values.pop_back();
        vp.emplace_back(Point(label, values, idx++));
    }

    return vp;
}

std::vector<Point> parseRandomGeneratedData(std::vector<std::vector<double>> generatedPoints) 
{
    std::vector<Point> vp{};
    const int n = generatedPoints.size();
    for (int i=0; i<n; i++) {
        vp.emplace_back(Point(-1, generatedPoints[i], i));
    }
    return vp;
}

void DBSCANfromDataset::run() {
    Point::resetDimension();
    auto points(normalize(parseDataset(filename)));
    auto cluster(naive_dbscan(points, eps, minPts));
    print_clusters(points, cluster);
}

void DBSCANfromGenerator::run(std::vector<std::vector<double>> generatedVectors) {
    Point::resetDimension();
    auto points(normalize(parseRandomGeneratedData(generatedVectors)));
    auto points_naive_clusters(naive_dbscan(points, eps, minPts));
    print_clusters(points, points_naive_clusters);
}