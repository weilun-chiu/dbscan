# C++ Parallelized Grid-based DBSCAN 

This project provides an optimized implementation of the Density-Based Spatial Clustering of Applications with Noise (DBSCAN) algorithm in C++. DBSCAN is a popular unsupervised machine learning algorithm used for clustering and outlier detection. The project includes a naive approach and a grid-based approach, and utilizes parallelism to improve the algorithm's performance.

## Table of Contents

- [Introduction](#introduction)
- [Usage](#usage)
- [Example](#Example)
- [Milestones](#milestones)
- [Parallelism](#Parallelism)
- [Futureworks](#Futureworks)
- [Contributing](#contributing)
- [Log](#Log)

## Introduction

The aim of this project is to provide an efficient and scalable implementation of the DBSCAN algorithm in C++. The grid-based approach is optimized by using a 2D grid to index data points and speeding up the neighbor search process. Parallelism is utilized to accelerate the algorithm's performance, including OpenMP and C++ Concurrency API. Additionally, we have used AVX instructions to optimize floating-point arithmetic operations in the program.

## Usage

To make use of this project, you can add the DBSCAN header file #include "dbscan.h" to your C++ project and create instances of the provided dbscan object types listed below. These objects take your data as an input parameter, and the result will be a vector containing cluster labels for every point in your dataset.

* NaiveDBSCAN
* SerialGridDBSCAN
* OMPGridDBSCAN
* ConcurrencyGridDBSCAN
* ConcurrencyStealingGridDBSCAN
* ConcurrencyStealingAVX2GridDBSCAN

## Example

The `main.cpp` file serves as the primary example of usage, while the `test.cpp` file exhibits the results of the unit tests.

```C++
    // Data generations
    DataGenerator generator(2, 100000);
    auto cluster_data = generator.cluster_distribution(3);
    
    SerialGridDBSCAN serial_grid_dbscan(eps, minPts);
    serial_grid_dbscan.run(cluster_data);

    OMPGridDBSCAN omp_grid_dbscan(eps, minPts);
    omp_grid_dbscan.run(cluster_data);

    ConcurrencyGridDBSCAN concurrency_grid_dbscan(eps, minPts);
    concurrency_grid_dbscan.run(cluster_data);

    ConcurrencyStealingGridDBSCAN concurrency_stealing_grid_dbscan(eps, minPts);
    concurrency_stealing_grid_dbscan.run(cluster_data);
```

## Milestones

This project has three milestones:

- [x] **Naive Approach**: Implement DBSCAN using a naive approach and benchmark its performance.
- [x] **Grid-Based Approach**: Optimize the algorithm by using a grid-based approach for neighbor search and benchmark its performance.
- [x] **Parallelization**: Parallelize the algorithm using OpenMP to speed up computation and benchmark its performance.
- [x] **Parallelization**: Parallelize the algorithm using C++ Concurrency API to speed up computation and benchmark its performance.

## Parallelism

### Optimizing Performance: Addressing the 99% Execution Time in 1% of the Code

| Procedure              | Runtime    | Percentage |
| ----------------------| ---------- | -----------|
| assignPoints           | 0.011691   | 0.9538%    |
| mark_ingrid_corecell   | 1.68E-06   | 0.0001%    |
| mark_outgrid_corecell  | 0.00318535 | 0.2599%    |
| expand                 | 1.21085    | 98.7861%   |

Based on the provided data, it can be observed that the "expand" procedure takes up the majority of the runtime with 98.7861%. This indicates that improving the performance of the "expand" procedure should be the primary target for optimizing the overall performance of the system.

Meanwhile, the other procedures such as "assignPoints" and "mark_outgrid_corecell" take up a relatively small portion of the runtime, at 0.9538% and 0.2599% respectively. The "mark_ingrid_corecell" procedure takes up a negligible amount of runtime at 0.0001%.

Therefore, optimizing the "expand" procedure is likely to yield the most significant improvements in the system's overall performance.

![alt text](https://i.imgur.com/cVEfCsF.png "Serial Runtime Profiling")

### OpenMP and Analysis of Parallelism Bottleneck
Our initial attempt to parallelize the grid-based DBSCAN algorithm(n:50000, cluster:3, eps:0.2, minPts:2) showed that we achieved close to 2x speedup using 2 threads, but we didn't observe further benefits with more threads. Parallelizing grid-based DBSCAN is challenging due to load balancing, communication overhead, and maintaining correctness, but can be done with careful consideration.

![alt text](https://i.imgur.com/Qk3rBbz.png "Loop-based Parallism on OpenMP. vertical axis is execution time, horizon axis it number of threads.")
![alt text](https://i.imgur.com/Xy2ul3G.png "Task-based Parallism on OpenMP. vertical axis is execution time, horizon axis it number of threads.")

This perf report shows the percentage of CPU cycles spent on different functions during the execution of the dbscan program. The dist function in the dbscan code accounts for 27.50% of the CPU cycles. The malloc function in the libc library accounts for 29.23% of the CPU cycles, followed by _int_free and isConnect. The report also shows that the operator new function in the libstdc++ library and the cfree function in the libc library account for a significant portion of the CPU cycles. This suggests that memory allocation and deallocation may be a bottleneck in the program's performance.

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

### C++ Concurrency API

C++ concurrency API is a set of tools and libraries added to the C++ Standard Library to support writing concurrent and parallel code in C++. It was introduced in the C++11 standard, which was released in 2011. The API includes features like threads, mutexes, condition variables, futures, promises, and atomics, which allow developers to create and manage threads, protect shared resources, coordinate task execution, and synchronize asynchronous operations. The API is designed to be portable and efficient, but requires careful attention to detail and a good understanding of concurrency concepts.

here are a few examples of what C++ concurrency can do that OpenMP cannot:

- Fine-grained control over thread creation and management: C++ concurrency allows developers to create and manage threads at a lower level of abstraction than OpenMP. This can be useful when a project requires fine-tuned control over the number of threads created, the affinity of threads to specific cores, or the lifecycle of threads.

- Custom synchronization primitives: C++ concurrency provides a rich set of synchronization primitives, such as mutexes, condition variables, and semaphores, which can be used to protect shared resources and coordinate the execution of tasks. These primitives offer more flexibility and control than the OpenMP synchronization constructs, which are based on atomic operations and barriers.

- Custom memory management: C++ concurrency allows developers to customize the memory allocation and deallocation strategies used by their concurrent code. This can be useful when a project needs to optimize memory usage or avoid contention on shared memory pools.

- Custom scheduling policies: C++ concurrency provides a flexible framework for implementing custom thread scheduling policies, which can be used to optimize the performance of specific applications or workloads. OpenMP, in contrast, provides a fixed set of scheduling policies that may not be optimal for all situations.

Overall, the ability to finely control thread creation, synchronization, memory management, and scheduling is a key advantage of C++ concurrency over OpenMP, especially for projects that require high-performance and low-level optimization.

As a results, we choose to implement the C++ concurrency API to check if the performance can be further improved. We test the performance on this environment settings:(n:100000, cluster:3, eps:0.12, minPts:5). 

We have observed that the performance of C++ concurrency is marginally better than OpenMP, but there is a significant performance drop (3x slower) when the number of threads is 32. While performance improves with more threads, it worsens when the number of threads exceeds 64, which is due to the limited number of core cells (63). When the number of threads is 32, there is extreme workload imbalance where each thread, except the last one, is assigned two core cells. However, the last thread is assigned 32 core cells. Similarly, when the number of threads exceeds 63, the last thread takes all the workload, while others have none.

![alt text](https://i.imgur.com/klX6k6C.png "Comparison between tasked-based OpenMP and C++ Concurrency API. vertical axis is execution time, horizon axis it number of threads."))

### Dynamic Scheduling: Work-Stealing Technique

In order to tackle workload imbalances, we have introduced a work-stealing technique that allocates a task queue to every thread. This method allows the master thread to keep track of each worker thread's progress. When the master identifies a vacant task queue, it reassigns a task from a neighboring queue to the idle thread for processing. The work-stealing approach guarantees effective utilization of threads, fostering equitable workload distribution and boosting overall performance. The impact of this strategy is illustrated in the accompanying figure. While there is a minor overhead with a lower thread count, the work-stealing method outperforms other frameworks once the number of threads reaches 25 or more.

![alt text](https://i.imgur.com/TOE4IJX.png "Comparison between tasked-based OpenMP, C++ Concurrency API, and C++ Concurrency API with work-stealing. vertical axis is execution time, horizon axis it number of threads.")

### Accelerating Parallel Programs with SIMD and AVX Instructions
In our previous section, we discovered that the primary performance bottleneck in our parallel program was related to memory allocation and floating-point arithmetic operations. In an effort to further optimize our program, we turned to the study of SIMD and AVX instructions. We implemented a new version of our parallelism framework that enabled AVX instructions for the most critical floating-point arithmetic functions. We test the performance on this environment settings:(n:10000000, cluster:3, eps:0.12, minPts:5). The results demonstrate that we were able to achieve significant performance gains - up to 16.27x on a single thread, 8.69x on 8 threads, and 35.89x compared to the serial program, details can be found in the following table and figure. These results demonstrate the tremendous potential of using SIMD and AVX instructions to accelerate parallel programs, particularly those with significant floating-point arithmetic computations.

| Threads | OpenMP | Concurrency + stealing | Concurrency + stealing + AVX |
|--------|---------------|-------------------------------|----------------------------------|
| 1	 |  55.780 (1.06)|  59.260 (1.00)				 |	3.642 (16.27)					|
| 4	 |  48.365 (1.23)|  20.842 (2.84)				 |	1.977 (29.98)					|
| 8	 |  21.455 (2.76)|  14.355 (4.13)				 |	1.651 (35.89)					|
| 16	 | 	17.445 (3.40)| 	14.128 (4.19) 				 |	1.695 (34.97)					|
| 32	 | 	17.340 (3.42)| 	14.672 (4.04) 				 |	1.757 (33.72)					|
| 64	 | 	14.798 (4.00)| 	14.930 (3.97) 				 |	1.749 (33.89)					|
| 78	 | 	15.292 (3.88)| 	15.482 (3.83) 				 |	1.721 (34.44)					|


![alt text](https://i.imgur.com/QTNiIJU.png "Comparison between tasked-based OpenMP, C++ Concurrency API with work-stealing, and C++ Concurrency API with work-stealing and AVX instructions. vertical axis is execution time, horizon axis it number of threads.")

### Conclusion
In summary, we were able to achieve a remarkable 12.79x improvement in runtime through our implementation of parallelism techniques, which involved identifying and optimizing performance hotspots that accounted for 99% of the total runtime. Our approach included profiling the runtime, using OpenMP for initial parallelism performance, and optimizing further with the C++ Concurrency API and work-stealing mechanism. Additionally, we explored the use of SIMD and AVX instructions to further accelerate the most critical floating-point arithmetic functions, which led to significant performance gains of up to 16.27x on a single thread, 8.69x on 8 threads, and 35.89x compared to the serial program.

## Futureworks

One of the main drawbacks of DBSCAN in high dimensional space is the "curse of dimensionality." As the number of dimensions increases, the distance between points tends to become more uniform, and the density-based approach used by DBSCAN becomes less effective.  To overcome these limitations, KDTree can be used as an alternative approach to improve the performance of DBSCAN in high dimensional space. KDTree is a data structure that partitions data points into a tree structure based on their spatial proximity. This allows for efficient nearest neighbor searches, which are essential for density-based clustering algorithms like DBSCAN.

- [ ] **KDtree Optimization**: Implement KDtree optimization to further improve search time and benchmark its performance.
- [ ] **Parallel KDtree**: Parallelize the building and searching of the KDtree to further speed up computation and benchmark its performance.

## Contributing

We welcome contributions to this project! If you would like to contribute, please submit a pull request with your changes.

## Log
- 5/16/2023:
    - We plan to develop the KDTree in another repository. As a result, we remove the original kdtree files.
    - Add const.
    - Add noexcept.
- 5/18/2023:
    - Add internal linkage in dbscan implementation for performance optimization.
    - Add `[[Likely]]` and `[[unlikely]]` on Point constructor for performanze optimization.
- 5/19/2023:
    - Add more `[[Likely]]` and `[[unlikely]]` to corner branches in Point header.
    - Review pure function in dbscan implementation.
    - Optimize the AVX implementation, remove unneccessary copy in dbscan implementation.
    - Mark the pure functions with `[[gnu::const]]` and `[[gnu::pure]]`.
