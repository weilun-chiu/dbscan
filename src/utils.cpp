#include "utils.h"
#include "point.h"
#include <vector>
#include <cmath>
#include <tuple>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>

double dist2(std::vector<double> const& lhs, std::vector<double> const& rhs) {
    double res{0.0f};
    for (int i=0; i<Point::dimensionality; i++) res += pow(lhs[i] - rhs[i], 2);
    return res;
}

double dist(std::vector<double> const& lhs, std::vector<double> const& rhs) {
    return sqrt(dist2(lhs, rhs));
}

std::tuple<std::vector<double>, std::vector<double>> calculateMinMaxValues(std::vector<Point> const& points) {
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

std::vector<double> normalize_point(Point const& point, std::vector<double> const& max_values, std::vector<double> const& min_values) {
    std::vector<double> normalized_point(Point::dimensionality);
    for (int i = 0; i < Point::dimensionality; i++) {
        normalized_point[i] = (point[i] - min_values[i]) / (max_values[i] - min_values[i]);
    }
    return normalized_point;
}

std::vector<Point> normalize(std::vector<Point> const& points) {
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

std::vector<int> rangeQuery(std::vector<Point> const& points, int p, double eps) {
    std::vector<int> neighbors;
    for (int i = 0; i < points.size(); i++) {
        if (dist(points[p], points[i]) <= eps) {
            neighbors.push_back(i);
        }
    }
    return neighbors;
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

std::vector<Point> parseRandomGeneratedData(std::vector<std::vector<double>> const& generatedPoints) 
{
    std::vector<Point> vp{};
    const int n = generatedPoints.size();
    for (int i=0; i<n; i++) {
        vp.emplace_back(Point(-1, generatedPoints[i], i));
    }
    return vp;
}