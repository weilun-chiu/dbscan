#include "dbscan.h"
#include "random_dataset.h"
#include "utils.h"


int main(int argc, char* argv[]) {
    double eps{0.12};
    int minPts{5};
    std::tie(eps, minPts) = parseDBSCANArguments(argc, argv);

    DBSCANfromDataset A("iris_dataset.csv", eps, minPts);
    A.run();

    DataGenerator generator(2, 1000);
    DBSCANfromGenerator B(eps, minPts);
    auto cluster_data = generator.cluster_distribution(3);
    B.run(cluster_data);
    auto uneven_data = generator.uneven_distribution(3, 1);
    B.run(uneven_data);

    return 0;
}
