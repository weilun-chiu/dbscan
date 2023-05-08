#pragma once

#include <vector>
#include <random>

class DataGenerator {
public:
    DataGenerator(int k, int n) : k_(k), n_(n) {}

    // Generate n evenly distributed k-dimensional vectors
    std::vector<std::vector<double>> even_distribution() {
        std::vector<std::vector<double>> data(n_, std::vector<double>(k_));
        for (int i = 0; i < k_; i++) {
            double step = 1.0 / (n_ + 1);
            for (int j = 0; j < n_; j++) {
                data[j][i] = (j + 1) * step;
            }
        }
        return data;
    }

    // Generate c dense clusters of k-dimensional vectors
    std::vector<std::vector<double>> cluster_distribution(int c) {
        std::vector<std::vector<double>> data(n_, std::vector<double>(k_));
        std::default_random_engine gen;
        std::normal_distribution<double> dist(0, 0.1);
        int points_per_cluster = n_ / c;
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < k_; j++) {
                std::normal_distribution<double> cluster_dist(i, 0.1);
                for (int p = 0; p < points_per_cluster; p++) {
                    int idx = i * points_per_cluster + p;
                    data[idx][j] = cluster_dist(gen) + dist(gen);
                }
            }
        }
        return data;
    }

    // Generate c dense clusters and u sparse clusters of k-dimensional vectors
    std::vector<std::vector<double>> uneven_distribution(int c, int u) {
        if (u == 0)
            return cluster_distribution(c);
        std::vector<std::vector<double>> data(n_, std::vector<double>(k_));
        std::default_random_engine gen;
        std::normal_distribution<double> dist(0, 0.1);
        int dense_points_per_cluster = n_ / (c + u);
        int sparse_points_per_cluster = n_ / (c + u);
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < k_; j++) {
                std::normal_distribution<double> cluster_dist(i, 0.1);
                for (int p = 0; p < dense_points_per_cluster; p++) {
                    int idx = i * dense_points_per_cluster + p;
                    data[idx][j] = cluster_dist(gen) + dist(gen);
                }
            }
        }
        for (int i = 0; i < u; i++) {
            for (int j = 0; j < k_; j++) {
                std::uniform_real_distribution<double> cluster_dist(0, 1);
                for (int p = 0; p < sparse_points_per_cluster; p++) {
                    int idx = c * dense_points_per_cluster + i * sparse_points_per_cluster + p;
                    data[idx][j] = cluster_dist(gen);
                }
            }
        }
        return data;
    }

private:
    int k_;
    int n_;
};