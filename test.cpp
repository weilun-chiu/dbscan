#include "src/point.h"
#include <iostream>
#include <cassert>
#include <cmath>

int main() {
    // Create a Point object
    std::vector<double> coords = {1.0, 2.0, 3.0};
    Point p(1, coords, 0);

    // Test the size() function
    assert(p.size() == 3);
    std::cout << "Size of point: " << p.size() << '\n';

    // Test the dist() function
    std::vector<double> coords2 = {2.0, 4.0, 6.0};
    double d = p.dist(coords2);
    assert(std::abs(d - 3.74166) < 1e-5);
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

    // Test the /= operator with zero divisor
    double divisorzero = 0.0;
    try {
        p /= divisorzero;
    } catch (const std::runtime_error& e) {
        std::cout << "Caught exception: " << e.what() << '\n';
    }
    std::cout << "New coordinates after division 0: " << p << '\n';

    std::cout << "All tests passed." << std::endl;
    return 0;
}
