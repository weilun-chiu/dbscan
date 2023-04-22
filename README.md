# C++ DBSCAN Implementation

This project provides a C++ implementation of DBSCAN (Density-Based Spatial Clustering of Applications with Noise), a popular clustering algorithm used in machine learning.

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Milestones](#milestones)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This project aims to provide an efficient and scalable implementation of DBSCAN in C++. It currently includes a naive approach and a grid-based approach, with plans to add parallelization and KDtree optimizations in future milestones.

## Installation

To install this project, you will need a C++ compiler and the Boost library installed on your machine. Once you have these dependencies installed, you can clone this repository and build the project using CMake.

## Usage

To use this project, you can include the DBSCAN header file in your C++ project and call the `dbscan` function with your data as input. The output will be a vector of cluster labels for each point in your dataset.

## Milestones

This project has five milestones:

1. **Naive Approach**: Implement DBSCAN using a naive approach and benchmark its performance.
2. **Grid-Based Approach**: Optimize the algorithm by using a grid-based approach for neighbor search and benchmark its performance.
3. **Parallelization**: Parallelize the algorithm using OpenMP to speed up computation and benchmark its performance.
4. **KDtree Optimization**: Implement KDtree optimization to further improve search time and benchmark its performance.
5. **Parallel KDtree**: Parallelize the building and searching of the KDtree to further speed up computation and benchmark its performance.

## Contributing

We welcome contributions to this project! If you would like to contribute, please submit a pull request with your changes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
