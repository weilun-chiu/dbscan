#include "utils.h"
#include "point.h"
#include <vector>
#include <cmath>

double dist2(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    double res{0.0f};
    for (int i=0; i<Point::dimensionality; i++) res += pow(lhs[i] - rhs[i], 2);
    return res;
}

double dist(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    return sqrt(dist2(lhs, rhs));
}