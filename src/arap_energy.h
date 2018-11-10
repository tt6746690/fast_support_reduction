#ifndef __ARAP_ENERGY_H__
#define __ARAP_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>


// Precompute data needed to compute ARAP energy
//
//      E(V', R) = 0.5 * tr(T^t * \tilde{L} * T) + tr(T^t * \tilde{K} * R)
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
    Eigen::SparseMatrix<double>& L,
    Eigen::SparseMatrix<double>& K);


// Computes ARAP energy given T, R
// notice that R are local rotation matrices, not a part of DOFs
//
//      E(V', R) = 0.5 * tr(T^t * \tilde{L} * T) + tr(T^t * \tilde{K} * R)
//      E(V', R) = 0.5 * tr(T^t * M^t * L * M * T) + tr(T^t * M^t * K * R)
// 
// Inputs:
//   T,  (d+1)m x d
//      vertical stack of transposed affine transformation for handles
//   M  n by m      n is number of vertices, m is number of handles
//       Linear blend skinning matrix `M` for computing `V' = M * T`
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
    const Eigen::MatrixXd& M,
    Eigen::SparseMatrix<double>& L,
    Eigen::SparseMatrix<double>& K);


#endif