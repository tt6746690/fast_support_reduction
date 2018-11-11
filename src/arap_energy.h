#ifndef __ARAP_ENERGY_H__
#define __ARAP_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <igl/min_quad_with_fixed.h>


// Precompute data needed to compute ARAP energy
//
//      E(V', R) = 0.5 * tr(T^t * \tilde{L} * T) + tr(T^t * \tilde{K} * R)
//
// Inputs:
//   V  #V by d vertex positions    d is dimension
//   F  #F by simplex-size list of element indices
//   M  n by m      n is number of vertices, m is number of handles
//       Linear blend skinning matrix `M` for computing `V' = M * T`
//   b  handle positions
// Outputs: 
//   L, (d+1)m by (d+1)m
//      L in above expression
//   K, (d+1)m by dn
//      K in above expression
//   data  
//      pre-factorized system matrix etc. (see `igl::min_quad_with_fixed`)
//
//      where d is dimension, 
//            m is number of handles
template<
    typename DerivedV,
    typename DerivedF,
    typename DerivedM,
    typename ScalarL,
    typename ScalarK>
void arap_precompute(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::MatrixBase<DerivedM>& M,
    Eigen::SparseMatrix<ScalarL>& L,
    Eigen::SparseMatrix<ScalarK>& K);

// Given precomputed data (`data` and `K`), handle _positions_ `bc` and current
// positions of all vertices `U`, conduct a _single_ iteration of the
// local-global solver for minimizing the as-rigid-as-possible energy. Output
// the _positions_ of all vertices of the mesh (by overwriting `U`).
//
// Inputs:
//   data  pre-factorized system matrix etc. (see `arap_precompute`
//   K  pre-constructed bi-linear term of energy combining rotations and
//     positions
//   U  #V by dim list of current positions (see output)
// Outputs:
//   U  #V by dim list of new positions (see input)
void arap_single_iteration(
    const igl::min_quad_with_fixed_data<double> & data,
    const Eigen::SparseMatrix<double> & K,
    const Eigen::MatrixXd & bc,
    Eigen::MatrixXd & U);

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
template<
    typename DerivedT,
    typename DerivedM,
    typename ScalarL,
    typename ScalarK>
double arap_energy(
    const Eigen::MatrixBase<DerivedT>& T,
    const Eigen::MatrixBase<DerivedM>& M,
    const Eigen::SparseMatrix<ScalarL>& L,
    const Eigen::SparseMatrix<ScalarK>& K);


#endif