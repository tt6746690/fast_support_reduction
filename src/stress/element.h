#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

class Element {
public:
    Element(const Vector4i& ids);
public:
    void constructStiffnessMatrix(const MatrixXd& V, const MatrixXd& E, vector<Triplet<double>>& triplets);
public:
    MatrixXd B;
    Vector4i nodesIds;
    double volume;
};