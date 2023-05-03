#pragma once

#include <vector>
#include <tuple>
#include "point.h"

double dist2(std::vector<double> const& lhs, std::vector<double> const& rhs);
double dist(std::vector<double> const& lhs, std::vector<double> const& rhs);
std::tuple<std::vector<double>, std::vector<double>> calculateMinMaxValues(std::vector<Point> const& points);
std::vector<Point> normalize(std::vector<Point> const& points);
std::vector<int> rangeQuery(std::vector<Point> const& points, int p, double eps);
std::vector<Point> parseDataset(std::string filename);
std::vector<Point> parseRandomGeneratedData(std::vector<std::vector<double>> const& generatedPoints);