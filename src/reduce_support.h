#ifndef __REDUCE_SUPPORT_H__
#define __REDUCE_SUPPORT_H__
#include <Eigen/Core>
#include <Eigen/Sparse>

// Given mesh (V, F), find minimizer of energy using particle swarm optimization
//      
//      E = E_{arap} + E_{overhang} + E_{overlap},      where \bx ∈ \R^{2dm}
// 
// Inputs:
//      V   #V x 3,
//      F   #F x 3,
//      W   $V x m(d+1)
//              linear blend skinning weight matrix
//      alpha_max       maximal self-supporting angle
//      dp              normalized printing direction
//      pso_iters       number of iterations for `pso`
//      pso_populations size of particle swarm `pso`
//
//  Outputs:
//      T   (d+1)m x d
//          handle transformations
//          Note d is 3 for both 2D/3D case
//      U   #V x 3.
//          deformed vertex location, under LBS with `T`
//      where   d is dimension
//              m is number of handles
//
//  Returns:
//      minimum energy function ...

float reduce_support(
    const Eigen::MatrixXf& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXf& W,
    double alpha_max,
    const Eigen::RowVector3f& dp,
    int pso_iters,
    int pso_population,
    Eigen::MatrixXf& T,
    Eigen::MatrixXf& U);



#endif