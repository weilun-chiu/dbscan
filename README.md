# C++ DBSCAN Implementation

This project provides a C++ implementation of DBSCAN (Density-Based Spatial Clustering of Applications with Noise), a popular clustering algorithm used in machine learning.

## Table of Contents

- [Introduction](#introduction)
- [Usage](#usage)
- [Milestones](#milestones)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This project aims to provide an efficient and scalable implementation of DBSCAN in C++. It currently includes a naive approach and a grid-based approach, with plans to add parallelization and KDtree optimizations in future milestones.

## Usage

In order to utilize this project, you can add the DBSCAN header file to your C++ project and invoke the dbscan function, passing in your data as an input parameter. The outcome will be a collection of cluster labels for every point in your dataset, returned as a vector.
The `main.cpp` file serves as the primary example of usage, while the `test.cpp` file exhibits the results of the unit tests.

## Milestones

This project has five milestones:

- [x] **Naive Approach**: Implement DBSCAN using a naive approach and benchmark its performance.
- [ ] **Grid-Based Approach**: Optimize the algorithm by using a grid-based approach for neighbor search and benchmark its performance.
- [ ] **Parallelization**: Parallelize the algorithm using OpenMP to speed up computation and benchmark its performance.
- [ ] **KDtree Optimization**: Implement KDtree optimization to further improve search time and benchmark its performance.
- [ ] **Parallel KDtree**: Parallelize the building and searching of the KDtree to further speed up computation and benchmark its performance.

## Contributing

We welcome contributions to this project! If you would like to contribute, please submit a pull request with your changes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
