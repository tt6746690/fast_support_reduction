#ifndef __ARAP_ENERGY_H__
#define __ARAP_ENERGY_H__
#include "minitrace.h"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/min_quad_with_fixed.h>
#include <igl/normalize_row_sums.h>
#include <igl/cotmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/polar_svd3x3.h>
#include <igl/fit_rotations.h>

#include <vector>
#include <utility>

typedef std::vector<std::pair<int, int>> lii;

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
//   L, (d+1)m by (d+1)m
//      L in above expression
//   K, (d+1)m by dn
//      K in above expression
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
    Eigen::SparseMatrix<ScalarK>& K)
{
    typedef typename DerivedV::Scalar ScalarV;
    typedef typename DerivedF::Scalar ScalarF;
    typedef Eigen::Matrix<ScalarV, 3, 1> RowVector3VT;
    typedef Eigen::Matrix<ScalarF, 3, 1> RowVector3FT;

    // Compute L
    igl::cotmatrix(V, F, L);


    // Compute K
    std::vector<Eigen::Triplet<ScalarV>> triplets;
    triplets.reserve(F.rows() * 3 * 3 * 3 * 2);

    int i, j, k;
    RowVector3FT f;
    RowVector3VT e;

    for (int a = 0; a < F.rows(); ++a) {
        f = F.row(a);
        // half edge in face `f`
        for (int b = 0; b < 3; ++b) {
            i = f(b%3);
            j = f((b+1)%3);
            k = f((b+2)%3);
            e = L.coeff(i, j) * (V.row(i) - V.row(j));
            // assign k' = {i,j,k} and d = {1,2,3} s.t.
            //      k_{i, 3k' + d} =  e^n_{ij}
            //      k_{j, 3k' + d} = -e^n_{ij}
            for (int d = 0; d < 3; ++d) {
                triplets.emplace_back(i, 3*i+d,  e(d));
                triplets.emplace_back(j, 3*i+d, -e(d));
                triplets.emplace_back(i, 3*j+d,  e(d));
                triplets.emplace_back(j, 3*j+d, -e(d));
                triplets.emplace_back(i, 3*k+d,  e(d));
                triplets.emplace_back(j, 3*k+d, -e(d));
            }
        }
    }

    int nv = V.rows();
    K.resize(nv, 3*nv);
    K.setFromTriplets(triplets.begin(), triplets.end());
    K = K / 6.0;
}


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
    typename DerivedV,
    typename DerivedT,
    typename DerivedM,
    typename DerivedF,
    typename ScalarL,
    typename ScalarK>
double arap_energy(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedT>& T,
    const Eigen::MatrixBase<DerivedM>& M,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::SparseMatrix<ScalarL>& L,
    const Eigen::SparseMatrix<ScalarK>& K,
    bool is3d)
{
    MTR_SCOPE_FUNC();
    typedef typename DerivedT::Scalar ScalarT;
    typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;
    typedef Eigen::Matrix<ScalarT, 3, 3> Matrix3T;
    typedef Eigen::Matrix<ScalarT, 3, 1> Vector3T;

    MatrixXT U, C;
    U = M * T;
    C = K.transpose() * U;


    MatrixXT R(C.cols(), C.rows());
    
    // construct matrix R
    const int size = U.rows();
    Matrix3T Ck, Rk;
    for (int k = 0; k < size; k++) {
        Ck = C.block(3 * k, 0, 3, 3);
        igl::polar_svd3x3(Ck, Rk);
        R.block(0, 3 * k, 3, 3) = Rk;
    }


    lii edge_indices;
    if (is3d) {
        edge_indices = lii{{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    } else {
        edge_indices = lii{{0, 1}, {1, 2}, {2, 0}};
    }

    double obj = 0;
    int a, b;
    double coeff;
    Matrix3T R_a;
    Vector3T new_vec, old_vec, old_vec_T, trans_old_vec, diff_vec;
    for (int i = 0; i < F.rows(); i++) {
        for (auto p : edge_indices) {
            a = F(i, p.first);
            b = F(i, p.second);
            R_a = R.block(0, 3 * a, 3, 3).transpose();
            new_vec = U.row(a) - U.row(b);
            old_vec = V.row(a) - V.row(b);
            old_vec_T = old_vec.transpose();
            coeff = L.coeff(a, b);
            trans_old_vec = R_a * old_vec_T;
            diff_vec = new_vec - trans_old_vec;
            double diff = coeff * diff_vec.norm() * diff_vec.norm() * 1.0 / 6;
            obj += diff;
        }
    }


    return obj;
}


#endif