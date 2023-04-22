#pragma once

#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

constexpr double divbyzero{0.000000001};

class Point {
public:
    int label;
    int id;
    int cluster_label;
    std::vector<double> coords;
    static size_t dimensionality;
    // CTORs
    Point(const int _label, const std::vector<double> vd, const int _id = -1): label{_label}, id{_id}, cluster_label{-1}, coords{vd} {
        if (dimensionality == 0) {
            dimensionality = coords.size();
        } else if (dimensionality != coords.size()) {
            throw std::invalid_argument("Invalid dimensionality.");
        }
    };
    Point() {};

    static void resetDimension() {dimensionality = 0;}
    static void setDimension(size_t d) {dimensionality = d;}

    // Overloads
    auto begin() const {
        return coords.begin();
    }
    auto end() const {
        return coords.end();
    }
    size_t size() const {
        return coords.size();
    }
    bool empty() const {
        return coords.empty();
    }
    operator std::vector<double>() const {
        return coords;
    }
    void resize(const size_t d) {coords.resize(d);}
    Point& operator+=(const std::vector<double> rhs) {
        for (int i = 0; i < dimensionality; i++) {
            coords[i] += rhs[i];
        }
        return *this;
    }
    Point& operator-=(const std::vector<double> rhs) {
        for (int i = 0; i < dimensionality; i++) {
            coords[i] -= rhs[i];
        }
        return *this;
    }
    Point& operator/=(const std::vector<double> rhs) {
        for (int i = 0; i < dimensionality; i++) {
            double div = rhs[i];
            if (div == 0) {
                div = divbyzero;
            }
            coords[i] /= div;
        }
        return *this;
    }
    Point& operator*=(const std::vector<double> rhs) {
        for (int i = 0; i < dimensionality; i++) {
            coords[i] *= rhs[i];
        }
        return *this;
    }
    Point& operator+=(double rhs) {
        for (int i = 0; i < dimensionality; i++) {
            coords[i] += rhs;
        }
        return *this;
    }
    Point& operator-=(double rhs) {
        for (int i = 0; i < dimensionality; i++) {
            coords[i] -= rhs;
        }
        return *this;
    }
    Point& operator/=(double rhs) {
        for (int i = 0; i < dimensionality; i++) {
            double div = rhs;
            if (div == 0) {
                div = divbyzero;
            }
            coords[i] /= div;
        }
        return *this;
    }
    Point& operator*=(double rhs) {
        for (int i = 0; i < dimensionality; i++) {
            coords[i] *= rhs;
        }
        return *this;
    }
    double operator[](const int i) const{
        return coords[i];
    }
    Point& operator=(Point rhs) {
        std::swap(coords, rhs.coords);
        label = rhs.label;
        id = rhs.id;
        cluster_label = rhs.cluster_label;
        return *this;
    }
    void operator=(std::vector<double> rhs) {
        std::swap(coords, rhs);
    }
    bool operator==(const std::vector<double> rhs) const{
        for (int i=0; i<dimensionality; i++)
            if (coords[i] != rhs[i])
                return false;
        return true;
    }
    bool operator!=(const std::vector<double>& rhs) const{
        return !(*this == rhs);
    }
    bool operator<(const std::vector<double>& rhs) const {
        if (dimensionality != rhs.size()) {
            throw std::invalid_argument("Vectors must be of same size");
        }
        for (int i = 0; i < dimensionality; i++) {
            if (coords[i] < rhs[i]) {
                return true;
            } else if (coords[i] > rhs[i]) {
                return false;
            }
        }
        return false;
    }
    friend std::ostream& operator<<(std::ostream& os, const Point& obj) {
        for (auto &p : obj.coords)
            os << p << " ";
        return os;
    }
    // Helper functions
    double dist2(const std::vector<double>& rhs) const {
        double res{0.0f};
        for (int i=0; i<dimensionality; i++) res += pow(coords[i] - rhs[i], 2);
        return res;
    }
    double dist(const std::vector<double>& rhs) const {
        return sqrt(dist2(rhs));
    }
};

