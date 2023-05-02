#include "dbscan.h"
#include "random_dataset.h"
#include "utils.h"


int main(int argc, char* argv[]) {
    // Parameters
    double eps{0.12};
    int minPts{5};
    int n{5};
    std::tie(eps, minPts) = parseDBSCANArguments(argc, argv);

    // NaiveDBSCAN naive_dbscan(eps, minPts);
    // naive_dbscan.run("iris_dataset.csv");

    // Data generations
    DataGenerator generator(2, n);
    auto cluster_data = generator.cluster_distribution(3);
    
    NaiveDBSCAN naive_dbscan(eps, minPts);
    naive_dbscan.run(cluster_data);

    SerialGridDBSCAN serial_grid_dbscan(eps, minPts);
    serial_grid_dbscan.run(cluster_data);

    // OMPGridDBSCAN omp_grid_dbscan(eps, minPts);
    // omp_grid_dbscan.run(cluster_data);

    ConcurrencyGridDBSCAN concurrency_grid_dbscan(eps, minPts);
    concurrency_grid_dbscan.run(cluster_data);

    ConcurrencyStealingGridDBSCAN concurrency_stealing_grid_dbscan(eps, minPts);
    concurrency_stealing_grid_dbscan.run(cluster_data);

    ConcurrencyStealingAVX2GridDBSCAN concurrency_stealing_AVX_grid_dbscan(eps, minPts);
    concurrency_stealing_AVX_grid_dbscan.run(cluster_data);

    return 0;
}
