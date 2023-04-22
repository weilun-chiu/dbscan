#pragma once

#include <vector>
#include <tuple>
#include "point.h"

double dist2(const std::vector<double>& lhs, const std::vector<double>& rhs);
double dist(const std::vector<double>& lhs, const std::vector<double>& rhs);
std::tuple<double, int> parseDBSCANArguments(int argc, char* argv[]);
std::tuple<std::vector<double>, std::vector<double>> calculateMinMaxValues(const std::vector<Point>& points);
std::vector<Point> normalize(const std::vector<Point>& points);
std::vector<int> rangeQuery(std::vector<Point>& points, int p, double eps);
std::vector<Point> parseDataset(std::string filename);
std::vector<Point> parseRandomGeneratedData(std::vector<std::vector<double>> generatedPoints);