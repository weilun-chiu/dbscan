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

Our initial attempt to parallelize the grid-based DBSCAN algorithm(n:50000, cluster:3, eps:0.2, minPts:2) showed that we achieved close to 2x speedup using 2 threads, but we didn't observe further benefits with more threads. Parallelizing grid-based DBSCAN is challenging due to load balancing, communication overhead, and maintaining correctness, but can be done with careful consideration.

![alt text](https://i.imgur.com/Qk3rBbz.png)
![alt text](https://i.imgur.com/Xy2ul3G.png)

This perf report shows the percentage of CPU cycles spent on different functions during the execution of the dbscan program. The dist function in the dbscan code accounts for 27.50% of the CPU cycles. The malloc function in the libc library accounts for 29.23% of the CPU cycles, followed by _int_free and isConnect. The report also shows that the operator new function in the libstdc++ library and the cfree function in the libc library account for a significant portion of the CPU cycles. This suggests that memory allocation and deallocation may be a bottleneck in the program's performance.

The perf static board shows that everything works as expected. 

```perf
Samples: 347K of event 'cycles:u', Event count (approx.): 282010497999
Overhead  Command  Shared Object        Symbol
  29.23%  dbscan   libc-2.28.so         [.] malloc     
  27.50%  dbscan   dbscan               [.] dist     
  16.92%  dbscan   libc-2.28.so         [.] _int_free   
   9.06%  dbscan   dbscan               [.] isConnect     
   5.84%  dbscan   libstdc++.so.6.0.28  [.] operator new   
   3.78%  dbscan   libc-2.28.so         [.] cfree@GLIBC_2.2.5   
   3.09%  dbscan   libc-2.28.so         [.] __memmove_avx_unaligned_erms  
   1.53%  dbscan   libstdc++.so.6.0.28  [.] operator delete@plt      
   1.12%  dbscan   dbscan               [.] memmove@plt          
   0.89%  dbscan   libstdc++.so.6.0.28  [.] malloc@plt        
```

The perf stat looks normal in terms of the performance counter statistics. It provides information on the time elapsed, CPU utilization, context-switches, page-faults, cycles, stalled-cycles-frontend and stalled-cycles-backend, instructions, branches, and branch-misses.

```perf
 Performance counter stats for './dbscan 0.2 2':

         87,602.85 msec task-clock:u              #    1.000 CPUs utilized          
                 0      context-switches:u        #    0.000 K/sec                  
                 0      cpu-migrations:u          #    0.000 K/sec                  
            11,303      page-faults:u             #    0.129 K/sec                  
   284,992,059,313      cycles:u                  #    3.253 GHz                      (50.00%)
        13,553,401      stalled-cycles-frontend:u #    0.00% frontend cycles idle     (50.00%)
     4,873,430,201      stalled-cycles-backend:u  #    1.71% backend cycles idle      (50.00%)
   925,491,792,090      instructions:u            #    3.25  insn per cycle         
                                                  #    0.01  stalled cycles per insn  (50.00%)
   226,426,881,223      branches:u                # 2584.698 M/sec                    (50.00%)
         1,155,082      branch-misses:u           #    0.00% of all branches          (50.00%)

      87.603927362 seconds time elapsed

      87.006668000 seconds user
       0.022818000 seconds sys
```


## Contributing

We welcome contributions to this project! If you would like to contribute, please submit a pull request with your changes.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
