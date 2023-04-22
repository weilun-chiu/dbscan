#pragma once

#include <vector>
#include <stdexcept>
#include <cmath>
#include <iostream>

class Point {
public:
    int label;
    int id;
    int cluster;
    std::vector<double> coords;
    static size_t dimensionality;
    // CTORs
    Point(const int _label, const std::vector<double> vd, const int _id = -1): label{_label}, id{_id}, cluster{-1}, coords{vd} {
        if (dimensionality == 0) {
            dimensionality = coords.size();
        } else if (dimensionality != coords.size()) {
            throw std::invalid_argument("Invalid dimensionality.");
        }
    };
    Point() {};
    // Overloads
    size_t size() const {return coords.size();}
    bool empty() const {return coords.empty();}
    operator std::vector<double>() const {
        return coords;
    }
    void resize(const size_t d) {coords.resize(d);}
    std::vector<double> operator-(const std::vector<double> rhs) const{
        std::vector<double> tmp(dimensionality);
        for (int i=0; i<dimensionality; i++)
            tmp[i] = coords[i] - rhs[i];
        return tmp;
    }
    std::vector<double> operator/(const std::vector<double> rhs) const{
        std::vector<double> tmp(dimensionality);
        for (int i=0; i<dimensionality; i++)
            tmp[i] = coords[i] / rhs[i];
        return tmp;
    }
    std::vector<double> operator*(const std::vector<double> rhs) const{
        std::vector<double> tmp(dimensionality);
        for (int i=0; i<dimensionality; i++)
            tmp[i] = coords[i] * rhs[i];
        return tmp;
    }
    double operator[](const int i) const{
        return coords[i];
    }
    void operator=(const std::vector<double> rhs) {
        coords = rhs;
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
        for (int i=0; i<dimensionality; i++) {
            if (coords[i] < rhs[i]) {
                return true;
            } else if (coords[i] > rhs[i]) {
                return false;
            }
        }
        return coords.size() < rhs.size();
    }
    friend std::ostream& operator<<(std::ostream& os, const Point& obj) {
        for (auto &p : obj.coords)
            os << p << " ";
        return os;
    }


    double dist2(const std::vector<double>& rhs) const {
        double res{0.0f};
        for (int i=0; i<dimensionality; i++) res += pow(coords[i] - rhs[i], 2);
        return res;
    }
    double dist(const std::vector<double>& rhs) const {
        return sqrt(dist2(rhs));
    }
};

