#pragma once

#include <Eigen/Core>

// generate vertices and edges for a centered square box
//      l       length of 1 side
void box(double l, Eigen::MatrixXd& V, Eigen::MatrixXi& F);


// implementation 

void box(double l, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    double h = l / 2;
    V.resize(8, 4);
    F.resize(4, 4);

    V <<
        -h, -h, -h, 1,
        h, -h, -h, 1,
        h, h, -h, 1,
        -h, h, -h, 1,
        -h, -h, h, 1,
        h, -h, h, 1,
        h, h, h, 1,
        -h, h, h, 1;

    F <<
        0, 1, 2, 3,
        4, 5, 6, 7,
        0, 4, 1, 5,
        2, 6, 3, 7;
}