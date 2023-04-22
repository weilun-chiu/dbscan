#include "dbscan.h"
#include "random_dataset.h"
#include "utils.h"


int main(int argc, char* argv[]) {
    // Parameters
    double eps{0.12};
    int minPts{5};
    std::tie(eps, minPts) = parseDBSCANArguments(argc, argv);
    // Data generations
    DataGenerator generator(2, 1000);
    auto cluster_data = generator.cluster_distribution(3);
    auto uneven_data = generator.uneven_distribution(3, 1);
    // Naive DBSCAN
    NaiveDBSCAN naive_dbscan(eps, minPts);
    naive_dbscan.run("iris_dataset.csv");
    naive_dbscan.run(cluster_data);
    naive_dbscan.run(uneven_data);
    // Grid DBSCAN
    GridDBSCAN grid_dbscan(eps, minPts);
    grid_dbscan.run("iris_dataset.csv");
    grid_dbscan.run(cluster_data);
    grid_dbscan.run(uneven_data);
    return 0;
}
