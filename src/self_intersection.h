#ifndef __SELF_INTERSECTION_H__
#define __SELF_INTERSECTION_H__
#include <Eigen/Sparse>
#include <Eigen/Core>

#define POS 1
#define NEG -1

// Return the self-intersecting area.
//
// Inputs:
//      V   #V by 3 list of vertices
//      F   #F by 3 list of triangle faces
double self_intersection(
    const Eigen::MatrixXd V,
    const Eigen::MatrixXi F,
    std::vector<Eigen::Vector3d> & intersection);

#endif