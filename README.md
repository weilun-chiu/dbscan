# C++ DBSCAN Implementation

This project provides a C++ implementation of DBSCAN (Density-Based Spatial Clustering of Applications with Noise), a popular clustering algorithm used in machine learning.

## Table of Contents

- [Introduction](#introduction)
- [Usage](#usage)
- [Example](#Example)
- [Milestones](#milestones)
- [Parallelism](#Parallelism)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This project aims to provide an efficient and scalable implementation of DBSCAN in C++. It currently includes a naive approach and a grid-based approach, with plans to add parallelization and KDtree optimizations in future milestones.

## Usage

To make use of this project, you can add the DBSCAN header file #include "dbscan.h" to your C++ project and create instances of the provided dbscan object types listed below. These objects take your data as an input parameter, and the result will be a vector containing cluster labels for every point in your dataset.

* NaiveDBSCAN
* GridDBSCAN(Only support dimension <= 2 now)

## Example

The `main.cpp` file serves as the primary example of usage, while the `test.cpp` file exhibits the results of the unit tests.

```C++
    // Data generations
    DataGenerator generator(2, 1000);
    auto cluster_data = generator.cluster_distribution(3);
    
    NaiveDBSCAN naive_dbscan(eps, minPts);
    naive_dbscan.run(cluster_data);
    
    GridDBSCAN grid_dbscan(eps, minPts);
    grid_dbscan.run(cluster_data);
```

## Milestones

This project has five milestones:

- [x] **Naive Approach**: Implement DBSCAN using a naive approach and benchmark its performance.
- [x] **Grid-Based Approach**: Optimize the algorithm by using a grid-based approach for neighbor search and benchmark its performance.
- [ ] **Parallelization**: Parallelize the algorithm using OpenMP to speed up computation and benchmark its performance.
- [ ] **KDtree Optimization**: Implement KDtree optimization to further improve search time and benchmark its performance.
- [ ] **Parallel KDtree**: Parallelize the building and searching of the KDtree to further speed up computation and benchmark its performance.

## Parallelism

Our initial attempt to parallelize the grid-based DBSCAN algorithm showed that we achieved close to 2x speedup using 2 threads, but we didn't observe further benefits with more threads. Parallelizing grid-based DBSCAN is challenging due to load balancing, communication overhead, and maintaining correctness, but can be done with careful consideration.

![alt text](https://i.imgur.com/Qk3rBbz.png)
![alt text](https://i.imgur.com/Xy2ul3G.png)

## Contributing

We welcome contributions to this project! If you would like to contribute, please submit a pull request with your changes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
