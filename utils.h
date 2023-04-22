#pragma once

#include <vector>
#include <tuple>

double dist2(const std::vector<double>& lhs, const std::vector<double>& rhs);
double dist(const std::vector<double>& lhs, const std::vector<double>& rhs);
std::tuple<double, int> parseDBSCANArguments(int argc, char* argv[]);