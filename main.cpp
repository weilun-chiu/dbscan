#include "dbscan.h"
#include "random_dataset.h"
#include "utils.h"


int main(int argc, char* argv[]) {
    // Parameters
    double eps{0.12};
    int minPts{5};
    std::tie(eps, minPts) = parseDBSCANArguments(argc, argv);
    // Data generations
    DataGenerator generator(2, 50000);
    auto cluster_data = generator.cluster_distribution(3);
    auto uneven_data = generator.uneven_distribution(3, 1);
    // Grid DBSCAN
    GridDBSCAN grid_dbscan(eps, minPts);
    grid_dbscan.run(cluster_data);
    return 0;
}
