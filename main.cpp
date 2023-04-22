#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <memory>
#include <limits>
#include <algorithm>
#include "point.h"
#include "kdtree.h"
#include <set>
#include "utils.h"


std::vector<Point> normalize(const std::vector<Point>& points) {
    std::vector<Point> result = points;

    if (points.empty()) {
        return result;
    }

    // Find the maximum and minimum values for each dimension
    std::vector<double> max_values(Point::dimensionality, std::numeric_limits<double>::min());
    std::vector<double> min_values(Point::dimensionality, std::numeric_limits<double>::max());
    for (const auto& point : points) {
        for (int i = 0; i < Point::dimensionality; i++) {
            max_values[i] = std::max(max_values[i], point[i]);
            min_values[i] = std::min(min_values[i], point[i]);
        }
    }

    // Normalize the data points
    for (int i = 0; i < points.size(); i++) {
        std::vector<double> tmpvd(Point::dimensionality);
        for (int j = 0; j < Point::dimensionality; j++) {
            tmpvd[j] = (points[i][j] - min_values[j]) / (max_values[j] - min_values[j]);
        }
        result[i] = tmpvd;
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


std::vector<int> cluster;
std::vector<int> neighborIdxs;
std::vector<Point> neighbors;
std::vector<int> subNeighborIdxs;
std::vector<Point> subNeighbors;
std::vector<int> visited;

std::vector<int> naive_dbscan(std::vector<Point>& points, double eps, int minPts) {
    visited.resize(points.size(), 0);
    cluster.resize(points.size(), -1);
    int clusterIdx = 0;

    for (int i = 0; i < points.size(); i++) {
        if (visited[i] == 1) continue;

        visited[i] = 1;
        neighbors.clear();
        neighborIdxs = rangeQuery(points, i, eps);
        if (neighborIdxs.size() < minPts) {
            cluster[i] = -1;
        } else {
            cluster[i] = clusterIdx;
            for (int j = 0; j < neighborIdxs.size(); j++) {
                int idx = neighborIdxs[j];
                if (visited[idx] == 0) {
                    visited[idx] = 1;
                    subNeighborIdxs.clear();
                    subNeighborIdxs = rangeQuery(points, idx, eps);
                    if (subNeighborIdxs.size() >= minPts) {
                        neighborIdxs.insert(neighborIdxs.end(), subNeighborIdxs.begin(), subNeighborIdxs.end());
                    }
                }
                if (cluster[idx] == -1) {
                    cluster[idx] = clusterIdx;
                }
            }
            clusterIdx++;
        }
    }

    std::cout << "Clusters: " << clusterIdx << std::endl;
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

    return cluster;
}

std::vector<int> kdtree_dbscan(std::vector<Point>& points, double eps, int minPts) {
    visited.resize(points.size(), 0);
    cluster.resize(points.size(), -1);
    int clusterIdx = 0;

    KDTree kdtree(points);

    // for (auto &p : points)
    //     std::cout << p.id << std::endl;
    ;
    for (int i = 0; i < points.size(); i++) {
        if (visited[i]) continue;
        visited[points[i].id] = true;

        neighbors.clear();
        neighbors = kdtree.search(points[i], eps);
        // std::cout << neighbors.size() << std::endl;

        if (neighbors.size() < minPts) {
            cluster[i] = -1;
        } else {
            cluster[i] = clusterIdx;
            for (auto & neighbor : neighbors) {
                // if (neighbor.size() < Point::dimensionality)
                //     neighbor.coords = points[neighbor.id];
                if (visited[neighbor.id] == false) {
                    visited[neighbor.id] = true;
                    subNeighbors.clear();
                    subNeighbors = kdtree.search(neighbor, eps);
                    if (subNeighbors.size() >= minPts) {
                        for (auto &sp : subNeighbors) {
                            if (visited[sp.id] == true)
                                continue;
                            // std::cout << sp << std::endl;
                            // std::cout << neighbors.size() << std::endl;
                            visited[sp.id] = true;
                            neighbors.push_back(sp);
                        }
                    }
                }
                if (cluster[neighbor.id] == -1) {
                    cluster[neighbor.id] = clusterIdx;
                }
            }
            // std::cout << "next" << std::endl;
            clusterIdx++;
        }
    }

    std::cout << "Clusters: " << clusterIdx << std::endl;
    for (int i = 0; i < points.size(); i++) {
        std::cout << "(";
        for (int j = 0; j < Point::dimensionality; j++) {
            std::cout << points[i][j];
            if (j < Point::dimensionality - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "): " << cluster[i] << std::endl;
    }

    return cluster;
}

std::vector<Point> parse(std::string filename) 
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        throw std::runtime_error("Error opening file");
    }

    std::string line{};
    std::vector<Point> vp{};
    int idx{0};
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        std::vector<double> values{};
        std::string token{};
        while (std::getline(iss, token, ',')) {
            double value{0};
            try {
                value = std::stod(token);
            } catch (const std::exception& e) {
                std::cerr << "Error parsing token " << token << ": " << e.what() << "\n";
                continue;
            }
            values.push_back(value);
        }
        int label = static_cast<int>(values.back());
        values.pop_back();
        Point p = Point(label, values, idx++);
        vp.push_back(p);
    }

    infile.close();
    return vp;
}

int main(int argc, char* argv[]) {

    double eps{0.12};
    int minPts{5};
    if (argc == 3)
    {
        eps = std::stof(argv[1]);
        minPts = std::stoi(argv[2]);
    }
    std::cout << "eps: " << eps << "  minPts: " << minPts << std::endl;

    auto points(normalize(parse("iris_dataset.csv")));
    // auto naive_clusters(naive_dbscan(points, eps, minPts));
    // std::cout << "KDTREE-impl" << std::endl;
    auto kdtree_clusters(kdtree_dbscan(points, eps, minPts));

    return 0;
}
