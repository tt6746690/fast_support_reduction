#pragma once
#include "minitrace.h"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/min_quad_with_fixed.h>
#include <igl/normalize_row_sums.h>
#include <igl/cotmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/polar_svd3x3.h>
#include <igl/fit_rotations.h>
#include <igl/columnize.h>

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

    // Compute L matrix
    igl::cotmatrix(V, F, L);


    // Compute K matrix
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



// Computes ARAP energy given K
// notice that R are local rotation matrices, not a part of DOFs
//
// E(V') = \frac{1}{2} \sum_{f\in \bF} \sum_{(i,j)\in f} c_{ijf} || (\bv_i' - \bv_j') - \bR_f(\bv_i - \bv_j) ||^2
// 
// Inputs:
//   V  #V by d vertex positions    d is dimension
//   T  (d+1)m x d
//      vertical stack of transposed affine transformation for handles
//   M  n by m      n is number of vertices, m is number of handles
//       Linear blend skinning matrix `M` for computing `V' = M * T`
//   F  #F by simplex-size list of element indices
//   L  (d+1)m by (d+1)m
//      cotangent matrix
//   K (d+1)m by dn
//      precomputed data matrix
//
//      where d is dimension, 
//            m is number of handles
//            r is number of edges
// Outputs:
//   arap energy


template<
    typename DerivedU,
    typename DerivedK,
    typename DerivedG,
    typename ScalarL,
    typename ScalarCSM>
double arap_energy(
    const Eigen::MatrixBase<DerivedU>& U,
    const Eigen::SparseMatrix<DerivedK>& K,
    const Eigen::MatrixBase<DerivedG>& G,
    const Eigen::SparseMatrix<ScalarL>& L,
    const Eigen::SparseMatrix<ScalarCSM>& CSM,
    bool is3d)
{

    MTR_BEGIN("C++", "arap");

    typedef typename DerivedU::Scalar ScalarT;
    typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;
    typedef Eigen::Matrix<ScalarT, 3, 3> Matrix3T;
    typedef Eigen::Matrix<ScalarT, 3, 1> Vector3T;

    const int n = U.rows();
    const auto & Udim = U.replicate(3, 1);
    MatrixXT S = CSM * Udim;
    const int Rdim = 3;
    MatrixXT R(Rdim, CSM.rows());
    MatrixXT Sn = S / S.array().abs().maxCoeff(); // normalize to prevent numerical issues

    igl::fit_rotations(Sn, true, R);

    const int num_rots = K.cols() / Rdim / Rdim;
    MatrixXT eff_R;
    eff_R.resize(Rdim, num_rots * Rdim);

    for (int r = 0; r < num_rots; r++) {
        eff_R.block(0, Rdim * r, Rdim, Rdim) = R.block(0, Rdim * G(r), Rdim, Rdim);
    }
    MatrixXT Rcol;
    igl::columnize(eff_R, num_rots, 2, Rcol);
    MatrixXT Bcol = - K * Rcol;
    MatrixXT Bc(n, 3);
    for(int c = 0; c < 3; c++) {
        Bc.block(0, c, n, 1) = Bcol.block(c * n, 0, n, 1);
    }

    Matrix3T M1 = U.transpose() * L * U; //VLV
    Matrix3T M2 = U.transpose() * Bc; // RKV

    ScalarT obj = - 0.5 * M1.trace() + M2.trace();

    MTR_END("C++", "arap");

    // debug
    // std::cout << R.block(0, 0, 3, 3) << std::endl;
    // std::cout << "M1 trace: " << M1.trace() << std::endl;
    // std::cout << "M2 trace: " << M2.trace() << std::endl;
    // std::cout << "L: " << L.rows() << " x " << L.cols() << std::endl;
    // std::cout << "R: " << R.rows() << " x " << R.cols() << std::endl;
    // std::cout << "CSM: " << CSM.rows() << " x " << CSM.cols() << std::endl;
    // std::cout << "S: " << S.rows() << " x " << S.cols() << std::endl;

    return obj;

}

