#ifndef __SELF_INTERSECTION_H__
#define __SELF_INTERSECTION_H__
#include <Eigen/Sparse>
#include <Eigen/Core>

// for 2D
#define POS 1
#define NEG -1

// for 3D
#define IN -1
#define OUT 1

// Return the self-intersecting area.
//
// Inputs:
//      V   #V by 3 list of vertices
//      F   #F by 3 list of triangle faces
double self_intersection_2d(
    const Eigen::MatrixXf Vf,
    const Eigen::MatrixXi F);

// Return the self-intersecting volume of a 3D mesh.
//
// Inputs:
//      V   #V by 3 list of vertices
//      F   #F by 3 list of triangle faces
double self_intersection_3d(
    const Eigen::MatrixXf Vf,
    const Eigen::MatrixXi F);

#endif