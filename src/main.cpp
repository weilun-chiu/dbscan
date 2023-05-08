#include "dbscan.h"
#include "random_dataset.h"
#include "utils.h"
#include <unistd.h>  // for getopt function

int main(int argc, char* argv[]) {
    // Parameters
    double eps{0.12};
    int minPts{5};
    int n{10000000};

    // Argparse
    int opt;
    while ((opt = getopt(argc, argv, "e:m:n:h")) != -1) {
        switch (opt) {
        case 'e':
            eps = atof(optarg);
            break;
        case 'm':
            minPts = atoi(optarg);
            break;
        case 'n':
            n = atoi(optarg);
            break;
        case 'h':
            std::cout << "Usage: " << argv[0] << " [-e eps] [-m minPts] [-n n]\n";
            return 0;
        default:
            std::cerr << "Invalid argument.\n";
            return 1;
        }
    }

    // NaiveDBSCAN naive_dbscan(eps, minPts);
    // naive_dbscan.run("iris_dataset.csv");

    // Data generations
    DataGenerator generator(2, n);
    auto cluster_data = generator.cluster_distribution(3);
    
    // NaiveDBSCAN naive_dbscan(eps, minPts);
    // naive_dbscan.run(cluster_data);

    // SerialGridDBSCAN serial_grid_dbscan(eps, minPts);
    // serial_grid_dbscan.run(cluster_data);

    OMPGridDBSCAN omp_grid_dbscan(eps, minPts);
    omp_grid_dbscan.run(cluster_data);

    // ConcurrencyGridDBSCAN concurrency_grid_dbscan(eps, minPts);
    // concurrency_grid_dbscan.run(cluster_data);

    ConcurrencyStealingGridDBSCAN concurrency_stealing_grid_dbscan(eps, minPts);
    concurrency_stealing_grid_dbscan.run(cluster_data);

    ConcurrencyStealingAVX2GridDBSCAN concurrency_stealing_AVX_grid_dbscan(eps, minPts);
    concurrency_stealing_AVX_grid_dbscan.run(cluster_data);

    return 0;
}
