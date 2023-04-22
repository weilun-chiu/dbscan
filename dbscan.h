#pragma once

#include <vector>
#include <string>
#include <functional>
#include "point.h"

class DBSCAN {
public:
    DBSCAN(double _eps, int _minPts) : eps(_eps), minPts(_minPts) {}
    virtual ~DBSCAN() {}
    virtual void run() = 0;
protected:
    double eps;
    int minPts;
};

class DBSCANfromDataset : public DBSCAN {
public:
    DBSCANfromDataset(const std::string& _filename, const double _eps, const int _minPts)
        : DBSCAN(_eps, _minPts), filename(_filename) {}
    void run() override;
private:
    std::string filename;
};

class DBSCANfromGenerator : public DBSCAN {
public:
    DBSCANfromGenerator(double _eps, int _minPts)
        : DBSCAN(_eps, _minPts) {}
    void run(std::vector<std::vector<double>>);
private:
    void run() override {};
};

