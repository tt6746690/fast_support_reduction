#ifndef __ARAP_ENERGY_H__
#define __ARAP_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>


// Precompute data needed to compute ARAP energy
//
//      E(V', R) = tr(T^t * \tilde{L} * T) + tr(T^t * \tilde{K} * R)
//
// Inputs:
//   V  #V by d vertex positions    d is dimension
//   F  #F by simplex-size list of element indices
//   M  n by m      n is number of vertices, m is number of handles
//       Linear blend skinning matrix `M` for computing `V' = M * T`
// Outputs: 
//   tL, (d+1)m by (d+1)m
//      \tilde{L} in above expression
//   tK, (d+1)m by dn
//      \tilde{K} in above expression
//
//      where d is dimension, 
//            m is number of handles
void arap_precompute(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& M,
    Eigen::MatrixXd& tL,
    Eigen::MatrixXd& tK);


// Computes ARAP energy given T, R
//
//      E(V', R) = tr(T^t * \tilde{L} * T) + tr(T^t * \tilde{K} * R)
// 
// Inputs:
//   T,  (d+1)m x d
//      vertical stack of transposed affine transformation for handles
//   R,  d x dr
//      horizontal stack of 3x3 rotation matrix for each edge
//   tL, (d+1)m by (d+1)m
//      \tilde{L} in above expression
//   tK, (d+1)m by dn
//      \tilde{K} in above expression
//
//      where d is dimension, 
//            m is number of handles
//            r is number of edges
// Outputs:
//   arap energy
double arap_energy(
    const Eigen::MatrixXd& T,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& tK,
    const Eigen::MatrixXd& tL);


#endif