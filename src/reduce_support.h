#ifndef __REDUCE_SUPPORT_H__
#define __REDUCE_SUPPORT_H__
#include <Eigen/Core>
#include <Eigen/Sparse>


class ReduceSupportConfig{
public: 
    ReduceSupportConfig();
public:

    // input mesh is 3D or 2D
    bool is3d;

    // overhang
    double alpha_max;
    Eigen::RowVector3f dp;

    // pso
    double rotation_angle;
    int pso_iters;
    int pso_population;

    // coefficients to energy 
    double c_arap;
    double c_overhang;
    double c_intersect;

    bool display;
};


// Given mesh (V, F), find minimizer of energy using particle swarm optimization
//      
//      E = E_{arap} + E_{overhang} + E_{overlap},      where \bx ∈ \R^{2dm}
// 
// Inputs:
//      V   #V x 3,
//      F   #F x 3,
//      C   (m+1) x 3          vertex position for joint (assume 1 tree)
//      BE  m x 2              hande edge indexed into C
//              linear blend skinning weight matrix
//      W   #V x m(d+1)
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
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXf& W,
    const ReduceSupportConfig& config,
    Eigen::MatrixXf& T,
    Eigen::MatrixXf& U);



#endif