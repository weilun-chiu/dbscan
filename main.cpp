#include "src/dbscan.h"
#include "src/random_dataset.h"
#include "src/utils.h"
#include <unistd.h>  // for getopt function

int main(int argc, char* argv[]) {
    // Default Parameters
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

    // Data generations
    DataGenerator generator(2, n);
    auto cluster_data = generator.cluster_distribution(3);

    OMPGridDBSCAN omp_grid_dbscan(eps, minPts);
    omp_grid_dbscan.run(cluster_data);

    ConcurrencyStealingGridDBSCAN concurrency_stealing_grid_dbscan(eps, minPts);
    concurrency_stealing_grid_dbscan.run(cluster_data);

    ConcurrencyStealingAVX2GridDBSCAN concurrency_stealing_AVX_grid_dbscan(eps, minPts);
    concurrency_stealing_AVX_grid_dbscan.run(cluster_data);

    return 0;
}
