#include "utils.h"
#include "point.h"
#include <vector>
#include <cmath>
#include <tuple>

double dist2(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    double res{0.0f};
    for (int i=0; i<Point::dimensionality; i++) res += pow(lhs[i] - rhs[i], 2);
    return res;
}

double dist(const std::vector<double>& lhs, const std::vector<double>& rhs) {
    return sqrt(dist2(lhs, rhs));
}

std::tuple<double, int> parseDBSCANArguments(int argc, char* argv[]) {
    double eps{0.12};
    int minPts{5};
    if (argc == 3) {
        eps = std::stof(argv[1]);
        minPts = std::stoi(argv[2]);
    }
    std::cout << "eps: " << eps << "  minPts: " << minPts << std::endl;
    return {eps, minPts};
}