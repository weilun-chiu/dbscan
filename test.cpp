#include "src/point.h"
#include <iostream>

int main() {
    // Create a Point object
    std::vector<double> coords = {1.0, 2.0, 3.0};
    Point p(1, coords, 0);

    // Test the size() function
    std::cout << "Size of point: " << p.size() << '\n';

    // Test the dist() function
    std::vector<double> coords2 = {2.0, 4.0, 6.0};
    double d = p.dist(coords2);
    std::cout << "Distance between points: " << d << '\n';

    // Test the += operator
    std::vector<double> increment = {1.0, 1.0, 1.0};
    p += increment;
    std::cout << "New coordinates: " << p << '\n';

    // Test the -= operator
    std::vector<double> decr = {1.0, 1.0, 1.0};
    p -= decr;
    std::cout << "New coordinates: " << p << '\n';

    // Test the /= operator
    double divisor = 2.0;
    p /= divisor;
    std::cout << "New coordinates after division: " << p << '\n';

    // Test the /= operator
    double divisorzero = 0.0;
    p /= divisorzero;
    std::cout << "New coordinates after division 0: " << p << '\n';

    return 0;
}
